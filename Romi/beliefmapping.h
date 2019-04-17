#ifndef _BeliefMapping_h
#define _BeliefMapping_h

#include "mapping.h"

#define VISITED_BIT 0b10000000
#define SEEN_BIT 0b01000000
#define CONFIDENCE_MASK 0b00111111
#define NUM_BITS 6
#define DEFAULT_PROB 0b00011111 // this is default value of 32
#define ALPHA 0.7

class BeliefMapper : public Mapper {
    public:
        void printMap();
        void resetMap();
        void printRawMap();

        void updateMapFeature(byte feature, int y, int x);
    private:
        byte update(byte feature, int eeprom_address);
};

void BeliefMapper::resetMap()
{

    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for (int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            
            if (eeprom_address > 1023)
            {
                Serial1.println(F("Error: EEPROM Address greater than 1023"));
            }
            else
            {
                EEPROM.update(eeprom_address, DEFAULT_PROB);
                
            }
        }
    }

}

void BeliefMapper::printMap() {
    Serial1.println("Belief Map");
    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            bool seen_bit = (value & SEEN_BIT) >> 6;
            bool visited_bit = (value & VISITED_BIT) >> 7;
            byte conf = value & CONFIDENCE_MASK;
            if (visited_bit) {
                Serial1.print((char)'*');
            } else if (conf > DEFAULT_PROB) {
                Serial1.print((char)'O');
            } else {
                Serial1.print((char) MAP_DEFAULT_FEATURE);
            }
            Serial1.print(" ");
        }
        Serial1.println("");
    }
}

void BeliefMapper::printRawMap() {
    Serial1.println("Raw Map");
    for (int i=0;i<MAP_RESOLUTION;i++)
    {
        for(int j=0;j<MAP_RESOLUTION;j++)
        {
            int eeprom_address = (i*MAP_RESOLUTION)+j;
            byte value;
            value = EEPROM.read(eeprom_address);//, value);
            bool seen_bit = (value & SEEN_BIT) >> 6;
            short conf = value & CONFIDENCE_MASK;
            if(!seen_bit) {conf = conf * -1;}
            Serial1.print(conf);
            Serial1.print(" ");
        }
        Serial1.println("");
    }
}


byte BeliefMapper::update(byte feature, int eeprom_address) {
    byte value;
    value = EEPROM.read(eeprom_address);//, value);
    value = value | SEEN_BIT; // always set seen bit
    if (feature == '*') {
        value = value | VISITED_BIT; // set first bit only
    } else if (feature == (byte)'O') { // Obstacle update
        byte confidence = value & CONFIDENCE_MASK;
        int new_conf = ((int) confidence) * ALPHA + 63 * (1-ALPHA);
        confidence = (byte) new_conf;
        value = (value & 0b11000000) | (confidence & 0b00111111);
    } else { // non-obstacle update
        byte confidence = value & CONFIDENCE_MASK;
        int new_conf = ((int) confidence) * ALPHA + 0 * (1-ALPHA);
        confidence = (byte) new_conf;
        value = (value & 0b11000000) | (confidence & 0b00111111);
    }

    return  value;
}

void BeliefMapper::updateMapFeature(byte feature, int y, int x)
{
    if (x > MAP_X || x < 0 || y > MAP_Y || y < 0)
    {
      Serial1.println(F("Warning: detected obstacle out of bounds."));
      return;
    }

    int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
    int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);  

    int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;  

    if (eeprom_address > 1023)
    {
        Serial1.println(F("Error: EEPROM Address greater than 1023"));
    }
    else
    {   
        feature = update(feature, eeprom_address);
        EEPROM.update(eeprom_address, feature);
    }
        

}
#endif