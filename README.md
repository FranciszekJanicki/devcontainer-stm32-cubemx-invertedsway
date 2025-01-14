Projekt regulacji automatycznej stalowartosciowej ukladu odwroconego wahadla.

Korzystamy z ukladu MPU6050, bazujacego na ukladzie GY-521. MPU6050 zawiera akcelerometr, zyroskop oraz czujnik temperatury.
MPU6050 korzysta z  magistali I2C w konfiguracji Slave, zapewniajacy Masterowi dostep do swoich rejestrow.
Dokumentacja dostepna od producenta opisuje okolo 70% funkcjonalnosci MPU6050, jednak znalezlismy w internecie autorskie 
dokumentacje uzupelnione o brakujace rejestry i dokumentacje, na ktorych bazie napisalismy kod do obslugi Digital Motion Processor'a w ktory
wyposazony jest MPU6050. Normalnie pomiary z akcelerometru sa bardzo zaszumione, natomiast zyroskop dryftuje w czasie, gdyz pomiar pozycji katowej
jest niebezposredni. Predkosc probkowania calego ukladu uzaleznilismy wlasnie od predkosci probkowania ukladu MPU6050, ktory zapewnia odpowiednie przerwania.
Konfiguracja MPU6050 jest w pelni dowolna, wystarczy przekazac wybrane wartosci konfiguracji do funkcji inicjalizujacej obiekt struktury MPU6050.

Korzystamy z ukladu L298N jako sterownika silnika pradu stalego, uklad zasilany jest napieciem 6V, dlatego napiecie PWM generowane przez STM32 w zakresie 0-3.3V skalowane jest na 0-6V
Zmienne sterujace uklad L298N takze jest w pelni konfigurowalny przez uzytkownika

Do pomiaru pozycji i predkosci katowej silnika korzystamy z wbudowanego w niego enkodera, ktorego impulsy zliczamy z uzyciem licznika hardware'owego na STM32

Uklad korzysta z takze w pelni konfigurowalnych zmiennych reprezentuajcych regulator PID, ktory oblicza sygnal sterujacy bazujac na zadanym kacie oraz zmierzonym kacie przez MPU6050
