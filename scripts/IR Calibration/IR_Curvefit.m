%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IR Sensor Calibration
% Robotic Systems 2019 - UoB
% Alexander Smith, MEng
%
% Results viewable through cftool( IR_Calibration )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Generate distance matrix

distance = [ 100 : 50 : 600 ] ;

%% IR results
% Accuracy defined at all points as within 10mm of expected value 

% Centre sensor %

reading_centre_1 = [ 687 , 483 , 381 , 326 , 284 , 254 , 230 , 207 , 193 , 171 , 157 ] ;
reading_centre_2 = [ 697 , 485 , 376 , 321 , 282 , 249 , 228 , 209 , 192 , 178 , 162 ] ;
reading_centre_3 = [ 689 , 480 , 379 , 320 , 285 , 253 , 230 , 210 , 195 , 181 , 169 ] ;

averaged_centre = ( reading_centre_1 + reading_centre_2 + reading_centre_3 ) / 3 ;
cftool( distance , averaged_centre ) ;
% Accurate between ~80 - 600mm, drifts past

% Left sensor %

reading_left_1 = [ 700 , 488 , 391 , 324 , 288 , 256 , 229 , 212 , 195 , 178 , 168 ] ; 
reading_left_2 = [ 705 , 500 , 387 , 322 , 285 , 252 , 232 , 214 , 198 , 182 , 172 ] ;
reading_left_3 = [ 703 , 503 , 387 , 323 , 287 , 255 , 231 , 213 , 200 , 185 , 171 ] ;

averaged_left = ( reading_left_1 + reading_left_2 + reading_left_3 ) / 3 ;
cftool( distance , averaged_left ) ;
% Accurate between ~90 - 500mm, drifts past


% Right sensor %

reading_right_1 = [ 692 , 508 , 392 , 327 , 284 , 256 , 223 , 202 , 192 , 178 , 171 ] ;
reading_right_2 = [ 699 , 501 , 393 , 325 , 287 , 253 , 230 , 205 , 193 , 180 , 173 ] ;
reading_right_3 = [ 694 , 500 , 390 , 324 , 287 , 254 , 229 , 206 , 193 , 180 , 169 ] ;

averaged_right = ( reading_right_1 + reading_right_2 + reading_right_3 ) / 3 ;
cftool( distance , averaged_right ) ;
% Accurate between ~75 to 600mm