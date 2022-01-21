/******************************************************************************/
/*                                                                            */
/* Laboratorni uloha c. 4 z predmetu IMP                                      */
/*                                                                            */
/* Mereni hodnoty napeti na potenciometru pomoci AD prevodniku                */
/*                                                                            */
/* Reseni vytvoril(a) a odevzdava: Ladislav Dokoupil xdokou14                 */
/*                                                                            */
/******************************************************************************/

#include "MKL05Z4.h"
#include <stdio.h>
#include <ctype.h>
#include <string.h>

/* Aktivacni konstanty jednotlivych pozic LED displeje (aktivace pomoci log. 1) */
#define D1 0x0800 //PTA11
#define D2 0x1000 //PTA12
#define D3 0x2000 //PTA13
#define D4 0x1000 //PTB12

/* Aktivacni konstanty hodnot cislic (7 segmentu a-g na PTA3-PTA9, sviti v log. 0) */
#define N0 0x0607 // "0"
#define N1 0x07CF // "1"
#define N2 0x0527 // "2"
#define N3 0x0587 // "3"
#define N4 0x04C8 // "4"
#define N5 0x0497 // "5"
#define N6 0x0417 // "6"
#define N7 0x07C7 // "7"
#define N8 0x0407 // "8"
#define N9 0x0487 // "9"

/* Pomocne konstanty */
#define ERR 0x5FF // "-"
#define NIC 0x7FF // nic

/* Maska desetinne tecky na PTA10 - DP sviti v log. 0 */
#define DOT_ON_MASK 0x03FF

#define DELAY_LEDD 2000

const unsigned int digit[] = { N0, N1, N2, N3, N4, N5, N6, N7, N8, N9 };

unsigned int index;

/* Funkce inicializace MCU */
void MCU_Init(void)
{
    SIM->COPC = SIM_COPC_COPT(0x00); // Deaktivace modulu WatchDog
    // Aktivace hodin ridicich modulu portu A a B
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK;
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; // Aktivace hodin pro ADC0

    // Nastaveni vyvodu LED displeje jako univerzalni GPIO pro SW rizeni
    PORTA->PCR[3] = PORT_PCR_MUX(0x01);  // seg A
    PORTA->PCR[4] = PORT_PCR_MUX(0x01);  // seg B
    PORTA->PCR[5] = PORT_PCR_MUX(0x01);  // seg C
    PORTA->PCR[6] = PORT_PCR_MUX(0x01);  // seg D
    PORTA->PCR[7] = PORT_PCR_MUX(0x01);  // seg E
    PORTA->PCR[8] = PORT_PCR_MUX(0x01);  // seg F
    PORTA->PCR[9] = PORT_PCR_MUX(0x01);  // seg G
    PORTA->PCR[10] = PORT_PCR_MUX(0x01); // seg DP

    PORTA->PCR[11] = PORT_PCR_MUX(0x01); // DIG1
    PORTA->PCR[12] = PORT_PCR_MUX(0x01); // DIG2
    PORTA->PCR[13] = PORT_PCR_MUX(0x01); // DIG3

    PTA->PDDR = GPIO_PDDR_PDD(0x3FF8);  // Konfigurace jako vystupni piny

    PORTB->PCR[0] = PORT_PCR_MUX(0x00);  // Potenciometr jako vstup ADC0
    PORTB->PCR[12] = PORT_PCR_MUX(0x01); // DIG4 na portu B jako GPIO
    PTB->PDDR = GPIO_PDDR_PDD(0x1000);   // Konfigurace DIG4 jako vystupu
}

/* Funkce realizujici zpozdeni formou aktivniho cekani */
void delay(long long bound) { long long i; for(i=0; i < bound; i++); }

/* Funkce display_val nastavuje vyvody LED pro prave aktivni pozici (index)  *
 * retezce val_str a cislici 7-seg. LED displeje. Retezec val_str obsahuje   *
 * hodnotu pro zobrazeni na displeji ve formatu 1.234 (des. tecka nepovinna) */
void display_val(char *val_str) {

    // Jsme-li s ukazovatkem mimo retezec nebo mimo displej,
    // nic nezobrazujeme a skoncime
    if (index >= strlen(val_str) || index > 4)
    {
        PTA->PDOR = GPIO_PDOR_PDO(NIC);
        return;
    }
    if (isdigit(val_str[index])) // Ma smysl zobrazovat pouze cislice
    {
        PTA->PDOR = GPIO_PDOR_PDO(digit[val_str[index]-'0']);
        index++;
        // Je-li za cislici desetinna tecka, aktivujeme ji
        if (val_str[index]=='.' || val_str[index]==',')
        {
            PTA->PDOR &= GPIO_PDOR_PDO(DOT_ON_MASK);
            index++;
        }
    }
    else // pokud narazime na neciselny symbol, zobrazime "-" (ERR)
    {
        PTA->PDOR = GPIO_PDOR_PDO(ERR);
        index++;
    }
}

void SysInit(void)  // Povoleni hodin do modulu, ktere budou pouzivany:
{
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0x01); // zdroj hodin do casovace TPM
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; // povoleni hodin do portu B (pro RGB LED)
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // povoleni hodin do casovace TPM0
}

// Inicializace portu s RGB LED jednotlive barevne slozky LED
// budou rizeni prislusnymi kanalovymi vystupy casovace TPM0
// a to konkretne signalem PWM, krery bude casovacem generovan.
// Cisla kanalu pro LED vizte schema laboratorniho pripravku.
void PortInit(void)
{
	PORTB->PCR[9] = PORT_PCR_MUX(0x02);  //  PTB9: TPM0_CH2 -> red
	PORTB->PCR[10] = PORT_PCR_MUX(0x02); // PTB10: TPM0_CH1 -> green
	PORTB->PCR[11] = PORT_PCR_MUX(0x02); // PTB11: TPM0_CH0 -> blue
}

// UKOLY K DOPLNENI JSOU OZNACENY CISLOVANYMI KOMENTARI  // 1.   // 2.  ATD.

// Doporucena hodnota preteceni casovace
#define OVERFLOW 0xFF
void Timer0Init(void)
{
    // 1. Vynulujte registr citace (Counter) casovace TPM0
	TPM0_CNT = 69;

    // 2. Nastavte hodnotu preteceni do Modulo registru TPM0 (doporucena hodnota
    //    je definovana vyse uvedenou konstantou OVERFLOW, pripadne experimentujte
    //    s jinymi hodnotami v pripustnem rozsahu zmenou hodnoty teto konstanty).
	TPM0_MOD = OVERFLOW;

    // 3. Nastavte rezim generovani PWM na zvolenem kanalu (n) casovace v ridicim
    //    registru TPM0_CnSC tohoto kanalu, konkretne:
    //    Edge-aligned PWM: High-true pulses (clear Output on match, set Output on reload),
    //    preruseni ani DMA requests nebudou vyuzivany.
	TPM0_C0SC = 0x28;
	TPM0_C1SC = 0x28;
	TPM0_C2SC = 0x28;

    // 4. Nastavte konfiguraci casovace v jeho stavovem a ridicim registru (SC):
    //    (up counting mode pro Edge-aligned PWM, Clock Mode Selection (01),
    //    Prescale Factor Selection (Divide by 8), bez vyuziti preruseni ci DMA.
    //    POZN.: !!!!! Budete-li masky v SC registru nastavovat postupne, je NUTNO
    //    toto provadet pri Clock Mode Selection = 00 (tj. v rezimu TPM disabled).
	TPM0_SC = 0x0b;


    // Dalsi ukoly k doplneni jsou uvedeny v hlavni smycce.
}





/*****************************************************************************
 *                            UKOLY K DOPLNENI:                              *
 *                                                                           *
 * 1. Doplnte konfiguraci AD prevodniku ADC0 podle nasledujici specifikace:  *
 *    - zpracovani namerene hodnoty v ramci obsluhy preruseni,               *
 *    - zdroj hodinoveho signalu: (Bus clock)/2, clock divider = 8,          *
 *    - rezim prevodu: kontinualni konverze, 8-bit,                          *
 *    - vysledek bude hardwarove prumerovan z 32 vzorku.                     *
 * Nezapomente povolit preruseni v systemu NVIC (inspirace viz 2. cviceni).  *
 * Registry ADC0, ktere se netykaji vyse uvedene konfigurace, nemodifikujte. *
 * DULEZITE: REGISTR ADC0_SC1A JE NUTNO NASTAVIT JAKO POSLEDNI, JELIKOZ TEN  *
 * AKTIVUJE CINNOST PREVODNIKU A POTE JIZ JEHO KONFIGURACI NELZE MENIT.      */
void ADC0_Init(void) {
	NVIC_ClearPendingIRQ(ADC0_IRQn);
	NVIC_EnableIRQ(ADC0_IRQn);
	ADC0_CFG1 = 0x61;
	ADC0_SC3 = 0b1111;
	ADC0_SC1A = 0x46;
}

/* 2. Vypocet a vlozeni vysledku do nize uvedeneho retezce result provedte    *
 * doplnenim obsluzneho podprogramu preruseni. Pro tyto potreby se velmi hodi *
 * funkce sprintf znama ze std. C, ktera vsak ne na vsech platformach byva    *
 * pouzitelna v plnem rozsahu (na velmi jednoduchych MCU dokonce ani nemame   *
 * podporu pro floating-point vypocty!). KDO BUDE CHTIT FUNKCI SPRINTF VYUZIT *
 * PRO VYPIS FP VYSLEDKU, MUSI SI V NASTAVENI PROJEKTU V KDS AKTIVOVAT PRO TO *
 * PODPORU, NAVOD VIZ ZDE: https://community.nxp.com/thread/442798            *
 * pripadne je mozne vyuzit vlastni pomocne implementace nebo se inspirovat   *
 * necim existujicim, napr. zde: https://stackoverflow.com/questions/12703307/
 * arm-none-eabi-gcc-printing-float-number-using-printf                       *
 *
 * Do tohoto retezce vlozte vyslednou hodnotu prepoctenou z ADC ve formatu
 * s presnosti na 2 desetinna mista v rozsahu napeti potenciometru. Pozor:
 * jedna se o standardni ceckovsky retezec, ktery musi byt ukoncen znakem \0! */
char result[10] = "xxxx";
void ADC0_IRQHandler(void) {
	uint32_t val = ADC0_RA;
	val *= 330;
	val /= 255;

	result[3] = val%10 + '0';
	result[2] = (val%100)/10 + '0';
	result[1] = ',';
	result[0] = (val%1000)/100 + '0';

	TPM0_C0V = ADC0_RA;
	TPM0_C1V = ADC0_RA;
	TPM0_C2V = ADC0_RA;

}
/******************************************************************************/

int main(void)
{
    MCU_Init();
    ADC0_Init();
    SysInit();
    PortInit();
    Timer0Init();

    while(1) {
        // Index do retezce result, odkud bude zpracovavana hodnota, ktera
        // se ma zobrazit na 7-segm. LED displeji. V kazdem pruchodu hlavni
        // smyckou postupne aktivujeme jednotlive pozice displeje a pomoci
        // funkce display_val zobrazujeme prislusne cislice z retezce result.
        index = 0;

        display_val(result); // cislice k zobrazeni
        PTA->PDOR |= GPIO_PDOR_PDO(D1); // aktivace prvni cifry
        delay(DELAY_LEDD);
        PTA->PDOR &= GPIO_PDOR_PDO(~D1); // deaktivace prvni cifry

        display_val(result); // cislice k zobrazeni
        PTA->PDOR |= GPIO_PDOR_PDO(D2); // aktivace druhe cifry
        delay(DELAY_LEDD);
        PTA->PDOR &= GPIO_PDOR_PDO(~D2);

        display_val(result); // cislice k zobrazeni
        PTA->PDOR |= GPIO_PDOR_PDO(D3); // aktivace treti cifry
        delay(DELAY_LEDD);
        PTA->PDOR &= GPIO_PDOR_PDO(~D3);

        display_val(result); // cislice k zobrazeni
        PTB->PDOR |= GPIO_PDOR_PDO(D4); // aktivace ctvrte cifry
        delay(DELAY_LEDD);
        PTB->PDOR &= GPIO_PDOR_PDO(~D4);
    }
}

