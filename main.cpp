/*
Projet - Ruche connecté :
EI2I 4 (II) 

*/


// librairies utilisées :
#include "mbed.h"
#include "arm_math.h"
#include "dsp.h"
#include "arm_common_tables.h"
#include "arm_const_structs.h"
#include "DHT.h"
#include "DavisAnemometer.h"
#include "HX711.h"
 
#define SAMPLES                 512             
/* 256 real party and 256 imaginary parts */
#define FFT_SIZE                SAMPLES / 2     
/* FFT size is always the same size as we have samples, so 256 in our case */

float32_t  Input[SAMPLES];
float32_t  Output[FFT_SIZE];

// Variable globales utilisé pour les differents parties du code.

bool       trig=0;         // sémaphore de blocage de l'échantillonnage
int        indice = 0;

// mesure de l'état du batterie
float      BatterieLevel[1000];
float      BatterieMoy = 0;  
float      TrueBatterieMoy = 0;
int        ResFinale = 0; 

// mesure de la temperature exterieir et interieur
int        mesure = 0;
float      h = 0.0f, c = 0.0f;
int        mesure11 = 0;
float      h11 = 0.0f, c11 = 0.0f;

// capteur de vent 
static     DavisAnemometer anemometer(A1 /* wind direction */, D3 /* wind speed */);
float      anemo;

// capteur poids
long       valeur;
long       valeurTare;
float      raw;
float      p;
float      poids;

// affichage sigfox et envoie des message
int        AffichageSigfox = 0;
int        MaxSigfox = 0;

int        TriggerBoutton ;

// declaration des entrées sorties
DigitalIn  BouttonEtat(D10);
AnalogIn   myADC(A5);
AnalogOut  myDAC(A3);
AnalogIn   ain(A6); 


DHT        mydht(PA_0, DHT22);
DHT        mydht11(PA_3, DHT11);


AnalogIn   data(A1);
DigitalOut clk(D5);


HX711      Balance(A1,D5);       // Déclaration de l'objet HX711


Serial     pc(SERIAL_TX, SERIAL_RX); // test sur putty
Serial     device(PA_9, PA_10); // tx, rx
Ticker     timer;

 
void sample(){
    
    if(indice < SAMPLES){
        Input[indice] = myADC.read() - 0.7f;    
        //Real part NB removing DC offset of 0.7 volts
        Input[indice + 1] = 0;                  
        //Imaginary Part set to zero
        indice += 2;
    }
    else{ trig = 0; }
    
}
 
int main() {
    

    pc.baud(9600);
    device.baud(9600);
    float maxValue;            // Max FFT value is stored here
    uint32_t maxIndex;         // Index in Output array where max value is
    //device.printf("AT$SF=000000");
    pc.printf("\nDebut Enregistrement\n");
    
    Balance.setScale(5000);
    //Balance.tare();
    //valeurTare = Balance.getValue();                                           // On récupère la valeur de la Tare
    anemometer.enable();
    
    
    while(1) {
        
        AffichageSigfox ++;
        TriggerBoutton = BouttonEtat.read();
        //pc.printf("\nBoutton non appuie\n");
        if(TriggerBoutton){
                
                pc.printf("\nBoutton appuie\n");
                // boucle pour limiter la quantité des messages envoyé pour ne pas deborder( max est 140 messages par jour)
                if(AffichageSigfox == 10000) {
                    AffichageSigfox = 0;
                    MaxSigfox ++; 
                }
               
                if(trig == 0){
                    timer.detach();
                    // Initialisation du calcul de la FFT
                    // NB utilisation de fonctions prédéfinies arm_cfft_sR_f32_lenXXX, où XXX est le nombre d'échantillons, ici 256
                    
                    // Traitement de niveau de batterie : 
                    // recuperer la valeur moyenne de plusieurs lecture 
                    for(int i=0; i < 999; i++){
                        float f=(ain.read()*1.8f)*100;
                        BatterieLevel[i] = f; 
                    }
                    
                    for(int i=0; i < 998; i++){
                        BatterieMoy = BatterieLevel[i] + BatterieLevel[i+1]; 
                    }
                    
                    if(BatterieMoy <= 10){
                        ResFinale = 0;
                    }
                    if(BatterieMoy > 10 && BatterieMoy <= 20){
                        ResFinale = 1;
                    }
                    if(BatterieMoy > 20 && BatterieMoy <= 30){
                        ResFinale = 2;
                    }
                    if(BatterieMoy > 30 && BatterieMoy <= 40){
                        ResFinale = 3;
                    }
                    if(BatterieMoy > 40 && BatterieMoy <= 50){
                        ResFinale = 4;
                    }
                    if(BatterieMoy > 50 && BatterieMoy <= 60){
                        ResFinale = 5;
                    }
                    if(BatterieMoy > 70 && BatterieMoy <= 80){
                        ResFinale = 6; 
                    }
                    if(BatterieMoy > 90){
                        ResFinale = 7;
                    }
                    
                    // determination du resultat finale pour envoyé sur sigfox
                    switch(ResFinale){
                    case 0 : TrueBatterieMoy = 10; break;
                    case 1 : TrueBatterieMoy = 15; break;
                    case 2 : TrueBatterieMoy = 20; break;
                    case 3 : TrueBatterieMoy = 30; break;
                    case 4 : TrueBatterieMoy = 40; break;
                    case 5 : TrueBatterieMoy = 50; break;
                    case 6 : TrueBatterieMoy = 60; break;
                    case 7 : TrueBatterieMoy = 80; break;
                    }
                    
                    
                    pc.printf("Battery level is: %.0f % \r\n",TrueBatterieMoy);
                   
                    // Mesure de TEMP : 
                    // Lecture de Temperature interieur : 
                    mesure = mydht.readData();
                    if (0 == mesure) {
                        c   = mydht.ReadTemperature(CELCIUS);
                        h   = mydht.ReadHumidity();
                        pc.printf("TEMP exterieur: %4.lf % \r\n",c);
                        pc.printf("Humidity exterieur : %4.lf % \r\n",h);

                    } 
                    
                    // lecture de la temperature exterieure :
                    mesure11 = mydht11.readData();
                    if (0 == mesure11) {
                        c11   = mydht11.ReadTemperature(CELCIUS);
                        h11   = mydht11.ReadHumidity();
                        pc.printf("TEMP interieur : %4.lf % \r\n",c11);
                        pc.printf("Humidity interieur : %4.lf % \r\n",h11);
                    }               
                    
                         
                    // Mesure Poids 
                
                    Balance.setScale(10000);
                  
                    p=Balance.getGram();
                    raw=data.read();
                    poids=25.2f*p/78300.0f ;
                
                    pc.printf("Poids  : %.3lf\r\n",abs(poids));    // Affichage du poids
                                
                    // Mesure Force Vent
                    
                    anemo = anemometer.readWindSpeed();
                    
                    pc.printf("la valeur de l'anemo est : %f\r\n",anemo);
                    
                    // wait_ms(2000);
                    
                    // FFT
                    arm_cfft_f32(&arm_cfft_sR_f32_len256, Input, 0, 1);
         
                    // Calcul de la FFT et stockage des valeurs dans le tableau Output
                    arm_cmplx_mag_f32(Input, Output, FFT_SIZE);
                    Output[0] = 0;
                    // Calcul la valeur maximale du spectre - maxValue - et son indice - maxIndex
                    arm_max_f32(Output, FFT_SIZE/2, &maxValue, &maxIndex);
                  
                    // Affichage du spectre sous forme d'un signal analogique entre 0 et 3V.
                    myDAC=1.0f;     // Impulsion de synchronisation
                    wait_us(20);    
                    myDAC=0.0f; 
        
        
                    // Affichage des différentes valeurs du spectre
                    for(int i=0; i < FFT_SIZE / 2; i++){
                        myDAC=(Output[i]) * 0.9f;   // Mise à l'échelle de 90% de 3.3V
                        //pc.printf("FFT Value = %lf, %d \r\n", myDAC.read(), i);
                        //device.printf("AT$SF=%02x\n\r", (int) myDAC.read());
                        wait_us(10);               // Durée de chaque palier
                    }
                    
                    myDAC=0.0f;
                    // Affichage de la valeur max et de son indice
                    pc.printf("Peak FFT = %lf, %d \r\n", maxValue, maxIndex);
                  
                    // envoie des données a ubidots pour que l'utilisateur peux les visualiser
                    if(MaxSigfox <= 30){
                        if (0 == mesure11) {
                        device.printf("AT$SF=%02x%02x%02x%02x%02x\n\r", (int) TrueBatterieMoy,(int) c11,(int) h11,(int) anemo, (int) poids);
                        } 
                    }
                       
                    // Réinitialisation du sémaphore
                    trig = 1;
                    indice = 0;
                    timer.attach_us(&sample,40);      //40us 25KHz sampling rate
            }else{
                pc.printf("\nBoutton non appuie\n");
                }
        }
    }
}