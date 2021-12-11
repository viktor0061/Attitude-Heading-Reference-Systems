# VTS-Thesis - Attitude Heading Reference Systems(AHRS)<br>
This repository contains my algorithm based on the research I made in the field of AHRSs for my school project as an Electrical and Electronics Engineer(BSc) student at Subotica Tech - College of Applied Sciences with the support of prof. Dr. Silvester Pletl, and as a scholarship student at ELTE Márton Áron Special College(funders), and wich research earned me a section first place at the 20th Hungarian Students Scientific Conference, in Vojvodina.<br><br>
Hungarian<br>
A kód október 8-ai állapotától kezdve történik a verziószámozás<br>
Version 0.0.0<br>
 &ensp; Az algoritmus a bevitt adatokból:<br>
   &ensp; &ensp; -Kalibrálja gyorsulásmérő X és Y tengelyeit és a giroszkóp X,Y,Z tengelyeit.<br>
   &ensp; &ensp; -Meghatározza a giroszkópból illetve a gyorsulásmérőből és magnetometerből számolt szögeket külön-külön,<br>
   &ensp; &ensp; &nbsp; majd elvégzi azok fúzióját, miután a magnetometer adatait a gyorsulásmérőből számolt Roll és Pitch<br>
   &ensp; &ensp; &nbsp; szögek segítségével az inerciarendszerből a referencia rendszerbe transzformálta.<br>
   &ensp; &ensp; Ily módon meghatározza a test orientációját Euler szögekkel.<br>
   
Version 0.0.1<br>
 &ensp; -Javítsa a bugot ami miatt hamis adatokat adott a CF a beforgatott gyro adatokkal, <br>
 &ensp; &nbsp; radiánok és szögek közötti rossz átváltások miatt.<br>
 
Version 0.1.1<br>
&ensp; -Implementálásra került egy funkció, ami folyamatossá teszi a magnetométerből számolt Yaw szöget.<br>

Version 0.1.2<br>
&ensp; -A következő hibák kerültek javításra:<br>
&ensp; &ensp; -A giroszkóp adatok nem megfelelő forgatása a transzformációs mátrix rossz implementálása miatt.<br>
&ensp; &ensp; -A Roll szögben fellépő késés a CF egyenleteiben lévő rossz előjel miatt.<br>
&ensp; -Az aktuális verzió képes:<br>
&ensp; &ensp; -A három Euler szög meghatározására.<br>
&ensp; -Nem képes:<br>
&ensp; &ensp; -90 fokos vagy annál nagyobb szögeket mérni a Roll és Pitch tengelyek mentén a referenciának használt gravitáció vektor miatt.<br>
Version 1.0.0<br>
&ensp; -Implementálásra került egy módszer, amivel meghatározható a gyorsulásmérő Z - tengely menti offsetje.<br>
&ensp; -Javításra került egy hiba, az eCompass egyenleteinek rossz implementálása miatt -a nevezőben nem jelentek meg a vektor összegek-.<br>
&ensp; -Trigonometrikus és algebrai egyenletek helyett az eCompass a vektor forgatást már mátrix egyenletekkel végzi.<br>
&ensp; -Eltávolítottam a funkciót, ami a giroszkóp mérések adatait transzformálja rekúrzívan -szükségtelennek ítéltem-.<br>
&ensp; -Implementálásra került egy szűrés a gyorsulás adatokon, amelyeket a pozíció számításhoz használ fel az algoritmus -dead reckoninggal-.<br>
&ensp; -Továbbá bekerült egy funkció, amely a szűrt adatokat transzformálja REF/NED rendszerbe és kétszeresen integrálja.<br>
&ensp; -Változtatások történtek továbbá az adatok megjelenítésénél is.<br>
<br><br><br><br>

English<br>
Version control starts w/ 8th of October, 2021<br>
Version 0.0.0<br>
 &ensp; The algorithm based on the input data:<br>
   &ensp; &ensp; -Calibrates the X and Y axles of the acceletormeter and the X,Y,Z axles of the gyroscope.<br>
   &ensp; &ensp; -Calculates the attitude/tilt(Pitch&Roll) based on gyro and accelerometer, separately<br>
   &ensp; &ensp; -Transforms the magnetometer measurements from bodyframe to reference frame, w/ accelerometer tilt<br>
   &ensp; &ensp; -Calculates heading/yaw from the transformed magnetometer data<br>
   &ensp; &ensp; -Performs sensorfusion on the filtered attitude and heading data<br>
   &ensp; &ensp; &nbsp; from the gyroscope, accelerometer & magnetometer data (Complementary Filter)<br>
   &ensp; &ensp; -Thus provides attitude and heading information in Euler angles.<br>
   
Version 0.0.1<br>
 &ensp; -Fixes a bug where the CF have given false data with the transformed gyroscope data<br>
 &ensp; &nbsp; due to the wrong use of degrees and radians.<br>
 
Version 0.1.1<br>
&ensp; -Implements a feature which makes the output of the magYaw continuous.<br>

Version 0.1.2<br>
&ensp; -The next bugs have been fixed:<br>
&ensp; &ensp; -Wrong transformation of gyroscope data due to the wrong implementation of the transformation matrix.<br>
&ensp; &ensp; -Delay in the Roll angle due to a wrong sign in the CF equations.<br>
&ensp; -The current version is able to:<br>
 &ensp; &ensp; -Determine the Euler angles.<br>
&ensp; -And is not able to:<br>
&ensp; &ensp; -Determine tilt angles greater than 90 degrees because the nature of the gravity vector, used as reference.<br>
Version 1.0.0<br>
&ensp; -Implemented a feature which can calibrate the Z axis of the accelerometer.<br>
&ensp; -Fixed an error in the eCompass equations - the denominators of the equations have been implemented in a way<br> 
&ensp; &ensp;that only small degrees could be calculated with accuracy-.<br>
&ensp; -Algebraic equations have been replaced with matrix equations.<br>
&ensp; -Removed the possible transformation of gyroscope data -It have been deemed unnecessary-.<br>
&ensp; -Implemented a filter for accelerometer data, used to calculate position.<br>
&ensp; -Implemented a function that transform the filtered accelerometer data and performm double integration.<br>
&ensp; -Changes have been made in the visualization of the data.<br>

