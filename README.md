# VTS-Thesis---State-Observers<br>
A kód október 8-ai állapotától kezdve történik a verziószámozás<br>
Version 0.0.0<br>
 &ensp; Az algoritmus a bevitt adatokból:<br>
   &ensp; &ensp; -Kalibrálja a giroszkópot és a gyorsulásmérő X és Y tengelyét.<br>
   &ensp; &ensp; -Meghatározza a giroszkópból illetve a gyorsulásmérőből és magnetometerből számolt szögeket külön-külön,<br>
   &ensp; &ensp; &nbsp; majd elvégzi azok fúzióját, miután a magnetometer adatait a gyorsulásmérőből számolt Roll és Pitch<br>
   &ensp; &ensp; &nbsp; szögek segítségével az inerciarendszerből a referencia rendszerbe transzformálta.<br>
   &ensp; &ensp; -Az Euler szögeket a referencia rendszerhez viszonyítva határozza meg.<br>
   
Version 0.0.1<br>
 &ensp; -Javítsa a bugot ami miatt hamis adatokat adott a CF a beforgatott gyro adatokkal, <br>
 &ensp; &nbsp; radiánok és szögek közötti rossz átváltások miatt.<br>
 
Version 0.1.1<br>
&ensp; -Implementálásra került egy funkció, ami folyamatossá teszi az atan2() függvény kimenetét, amely a Yaw szög számításához szükséges.<br>

Version 0.1.2<br>
&ensp; -A következő hibák kerültek javításra:<br>
&ensp; &ensp; -A giroszkóp adatok nem megfelelő forgatása a transzformációs mátrix rossz implementálása miatt.<br>
&ensp; &ensp; -A Roll szögben fellépő késés a CF egyenleteiben lévő rossz előjel miatt.<br>
&ensp; -Az aktuális képes:<br>
&ensp; &ensp; -A három Euler szög meghatározására a referencia rendszernek használt NED- és test koordináta rendszerek között.<br>
&ensp; -Nem képes:<br>
&ensp; &ensp; -90 fokos vagy annál nagyobb szögeket mérni a Roll és Pitch tengelyek mentén az atan2() függvények és a referenciának használt gravitáció vektor miatt.<br>
