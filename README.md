# Autoelektronika

Za otvaranje periferija se koriste batch skripte koje se nalaze u folderima od periferija i imaju iste nazive kao .exe fajlovi. 

Za led bar dobiju se dva stuba levi (plavi) je ulazni i kod njega donje tri diode su odabir moda, prva odgore je MONITOR, druga DRIVE, treca SPEED. Drugi stub pokazuje da sistem radi tj. donja dioda sija ako je upaljen neki od tri moda, a ako ne sija ni jedan nije odabran. Izlazne diode svetle periodom od 200ms ako je upaljen alarm, a on se pali ako je rashladna tecnost toplija od 110 stepeni ili ako su obrtaji preko 6000. 

Za 7 segmentni displej se dobija jedan displej od 10 cifara i prva cifra sa leva je mod: 1-MONITOR, 2-DRIVE, 3-SPEED. Ostatalih devet cifara su podeljene na 4 cifre za prvi i 4 cifre za drugi senzor dok je izmedju njih jedna cifra koja je uvek nula. Kada sistem ne radi(nije odabran nijedan mod) nule su prikazane na svim ciframa.

UniCom za senzore (Unit 0) se podesava sa AUTO odgovorom. Triger je XYZ, a senzorske informacije se salju kao \00\s1\s2\s3\s4\s5\s6\ff. S2 je temperatura rashladne tecnosti a s3,s4 su obrtaji, ovi su za testiranje alarma bitni. Ako postoji greska u slanju gasi se sistem.

UniCom za PC (Unit 1) kada se salju komande na njega menja prikaz na 7seg displeju zavisno od trazenog moda, MONITOR\0d DRIVE\0d SPEED\0d su ako je komanda predugacka sistem ostavlja staru vrednost na displeju, a ako je pogresno uneseno gasi sistem.
