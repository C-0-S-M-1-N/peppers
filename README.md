## Docs
https://javadoc.io/doc/org.firstinspires.ftc

## Keep things clean

- da-ti fork la repo ca sa lucrati si face-ti-va branch separat pentru dev si reafactor (daca e cazul) sau alte chestii experimentale
sa pastram `main` ca branch cu codu bun
- dupa ce va luati un task mutati-l la stage-ul corect ca sa stim cine ce face
- NEW -> IN PROGRESS -> READY -> (cineva se uita peste cod) IN REVIEW -> (si ii da merge) DONE
- intrebati pe discord ce nu stiti
- TESTATI CODUL!!!!!!!!!!!! (o sa fiti surprinsi ca nu merge din prima)
- incercati sa pastrati o [structura uniforma](##structra_uniforma)

## structura_uniforma

- fiecare componenta/parte e implementata de interfata `Part` (cu `void update()` si `void update_values()`)
- NU facem roadrunner custom
- faceti clasele cat mai accesibile si custom ca sa fie usor de lucrat cu ele (sa fie modulare sa le putem folosi in auto)
- controalele se implementeaza doar in `Parts`