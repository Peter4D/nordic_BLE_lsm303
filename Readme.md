# V0 - POC (Proof Of Concept)

Zapisovanje tezav verzij ter popravkov posamezne verzije

## V0.0.0.1 - Bug's and improvements

Bug's found on base of V0.0.0.1 release:

- [x] **Kalibracija dela samo v eno smer in sicer v desno…če hočem kalibrirat v levo ne zazna 360 stopinj obrata** [MitjaOsek]

  Optimizacija algoritma - sedaj deluje v vse smeri ne glede na kateri tocki se zacne calibracija. Mozen je tudi prehod v drugo podrocje za dopolnitev stevila vzorcev kalibracije za obmocje

- [x] **Tipka prižiga ledice ko jo spustiš…ideja je da je logika obratna in sicer da ko pritisneš tipko ledica zasveti kratek blink** [MitjaOsek]

  Optimizacija algoritma

- [ ] **Stanja kalibracije ne shranjuje v eprom - če vzameš napajanje je kalibracijo treba ponavljati kar ni ok** [MitjaOsek]

- [x] **Ob zelo hitrem obratu zaklepanje/odklepanje ni zaznano** [MitjaOsek]

  BUG: Optimizacija `app_state.inserted` algoritma in vrednosti ob calibraciji

  ​	Vpeljava razlicnih vrednosti za vstavitev kljuca in drugacnih za izstavitev kljuca

  ​	Prenos algoritma v hitro zanko

  ​	Vpeljava razlicnih skal - 36 obmocij za obcutljive algoritme in iz teh 36 tudi 3 obmocja za zakljepanje/odklepanje

  ​		3 obmocja za odklepanje/zaklepanje

  ​		36 obmocij za graficni prikaz kalibracije v aplikaciji, vstavitev/izstavitev kljuca, ...

  Prenos algoritma `update_area_id` & `check_area_changed3` v hitro zanko

## V0.0.0.2 - Test Scenarios Consideration

- [ ] **V kolikor kljuc ni pravilno vpet v samo napravo je lahko obnasanje vstavitve/izstavitve drugacno**

  Potrebno je testiranje v skrajnih legah vpetega kljuca (kot med kljucem in napravo ni vedno idealnih 180 stopinj)
  
- [ ] **Razlicne hitrosti kalibracije**

  Kalibracija kljuca z hitrim vrtenjem in pravtako kalibracija kljuca z pocasnim vrtenje


## V0.0.0.2 - Bug's and improvements

