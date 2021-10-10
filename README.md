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
  
