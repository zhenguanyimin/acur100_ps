#include "myMath.h"
//#include "atlbase.h"
//#include "atlstr.h"
#ifdef DEBUG_LOG
//HANDLE my_mutex;
char _log_str[MAX_SIZE_LOG_STR] = { 0 };
int _log_ptr = 0;
static int log_mutex = -1;
#endif

#ifdef ENABLE_OUTRESULT
FILE* out_fp = NULL;
char* g_outPath = NULL;
FILE* warningOut_fp = NULL;
char* g_warningOutPath = NULL;
FILE* logOut_fp = NULL;
char* g_logOutPath = NULL;
#endif

#define DUMMY_SCALE 100.f

int dummy_frameCnt = 0;

//dummy data////////////////////////////////////////////////////
#define MAX_STEPS_NUM 10
#define MAX_RAND_OFF 20
int g_nDummyDataUsedIndex=0;
int g_arr_nTimeSeq[MAX_STEPS_NUM]={1,4,7,100,121,180,200,250,260,300};
float g_arr_fDummyData[MAX_STEPS_NUM*10]= //x,vx,y,vy,z,vz,  range,azimuth,elevation,doppler
{
	527.902771,2.000000,844.821045,1.000000,-87.155739,3.000000,1000.000061,-58.000004,-5.000000,1.639159,562.079407,2732.132812,773.635864,-5695.813965,-256.230865,-13529.009766,989.999939,-54.000000,-15.000003,601.757080,556.401855,-3186.337891,765.821411,5070.661133,253.642670,54318.886719,980.000061,-54.000000,15.000001,16212.166016,696.288147,3547.328125,-626.940613,-8664.843750,-251.054474,-55621.250000,970.000061,42.000000,-15.000000,22542.542969,506.786652,-5713.058594,811.028198,25098.771484,-83.669510,57534.210938,960.000000,-58.000004,-5.000000,13173.594727,917.070557,7382.009766,-32.024807,-28528.125000,-245.878098,-58194.000000,950.000000,2.000000,-15.000001,23149.539062,841.855408,-8284.588867,-340.131653,24830.843750,-243.289902,58225.046875,940.000061,22.000000,-15.000001,-31474.197266,595.517700,7102.167969,709.710327,-19791.609375,81.054832,-56668.218750,930.000000,-50.000000,5.000000,-15494.706055,700.267151,-4588.179688,547.108643,15889.166016,-238.113525,49008.156250,920.000061,-37.999996,-15.000001,-6727.561035,878.457031,5657.320312,-30.676395,-19355.875000,-235.525330,-48992.593750,910.000000,2.000000,-15.000001,18793.931641
};
//////////////////////////////////////////////////dummy data///


//const float cosTable[360] = {     
//  -0.999847695156391270f, -0.999390827019095760f, -0.998629534754573830f,     
//  -0.997564050259824200f, -0.996194698091745550f, -0.994521895368273290f,     
//  -0.992546151641321980f, -0.990268068741570250f,     
//  -0.987688340595137660f, -0.984807753012208020f, -0.981627183447663980f,     
//  -0.978147600733805690f, -0.974370064785235250f, -0.970295726275996470f,     
//  -0.965925826289068200f, -0.961261695938318670f,     
//  -0.956304755963035440f, -0.951056516295153530f, -0.945518575599316740f,     
//  -0.939692620785908320f, -0.933580426497201740f, -0.927183854566787310f,     
//  -0.920504853452440150f, -0.913545457642600760f,     
//  -0.906307787036649940f, -0.898794046299167040f, -0.891006524188367790f,     
//  -0.882947592858926770f, -0.874619707139395740f, -0.866025403784438710f,     
//  -0.857167300702112220f, -0.848048096156425960f,     
//  -0.838670567945424160f, -0.829037572555041620f, -0.819152044288991580f,     
//  -0.809016994374947340f, -0.798635510047292940f, -0.788010753606721900f,     
//  -0.777145961456970680f, -0.766044443118977900f,     
//  -0.754709580222772010f, -0.743144825477394130f, -0.731353701619170460f,     
//  -0.719339800338651300f, -0.707106781186547460f, -0.694658370458997030f,     
//  -0.681998360062498370f, -0.669130606358858240f,     
//  -0.656059028990507500f, -0.642787609686539360f, -0.629320391049837280f,     
//  -0.615661475325658290f, -0.601815023152048380f, -0.587785252292473030f,     
//  -0.573576436351045830f, -0.559192903470746680f,     
//  -0.544639035015027080f, -0.529919264233204790f, -0.515038074910054270f,     
//  -0.499999999999999780f, -0.484809620246337000f, -0.469471562785890530f,     
//  -0.453990499739546750f, -0.438371146789077510f,     
//  -0.422618261740699330f, -0.406736643075800100f, -0.390731128489273600f,     
//  -0.374606593415912070f, -0.358367949545300270f, -0.342020143325668710f,     
//  -0.325568154457156420f, -0.309016994374947340f,     
//  -0.292371704722736660f, -0.275637355816999050f, -0.258819045102520850f,     
//  -0.241921895599667790f, -0.224951054343864810f, -0.207911690817759120f,     
//  -0.190808995376544800f, -0.173648177666930300f,     
//  -0.156434465040231040f, -0.139173100960065350f, -0.121869343405147370f,     
//  -0.104528463267653330f, -0.087155742747658235f, -0.069756473744125330f,     
//  -0.052335956242943620f, -0.034899496702500733f,     
//  -0.017452406437283477f, 0.000000000000000061f, 0.017452406437283376f,     
//  0.034899496702501080f, 0.052335956242943966f, 0.069756473744125455f,     
//  0.087155742747658138f, 0.104528463267653460f,     
//  0.121869343405147490f, 0.139173100960065690f, 0.156434465040230920f,     
//  0.173648177666930410f, 0.190808995376544920f, 0.207911690817759450f,     
//  0.224951054343864920f, 0.241921895599667900f,     
//  0.258819045102520740f, 0.275637355816999160f, 0.292371704722736770f,     
//  0.309016994374947450f, 0.325568154457156760f, 0.342020143325668820f,     
//  0.358367949545300380f, 0.374606593415911960f,     
//  0.390731128489273940f, 0.406736643075800210f, 0.422618261740699440f,     
//  0.438371146789077460f, 0.453990499739546860f, 0.469471562785890860f,     
//  0.484809620246337110f, 0.500000000000000110f,     
//  0.515038074910054380f, 0.529919264233204900f, 0.544639035015027200f,     
//  0.559192903470746790f, 0.573576436351046050f, 0.587785252292473140f,     
//  0.601815023152048270f, 0.615661475325658290f,     
//  0.629320391049837500f, 0.642787609686539360f, 0.656059028990507280f,     
//  0.669130606358858240f, 0.681998360062498480f, 0.694658370458997370f,     
//  0.707106781186547570f, 0.719339800338651190f,     
//  0.731353701619170570f, 0.743144825477394240f, 0.754709580222772010f,     
//  0.766044443118978010f, 0.777145961456970900f, 0.788010753606722010f,     
//  0.798635510047292830f, 0.809016994374947450f,     
//  0.819152044288991800f, 0.829037572555041620f, 0.838670567945424050f,     
//  0.848048096156425960f, 0.857167300702112330f, 0.866025403784438710f,     
//  0.874619707139395740f, 0.882947592858926990f,     
//  0.891006524188367900f, 0.898794046299167040f, 0.906307787036649940f,     
//  0.913545457642600870f, 0.920504853452440370f, 0.927183854566787420f,     
//  0.933580426497201740f, 0.939692620785908430f,     
//  0.945518575599316850f, 0.951056516295153530f, 0.956304755963035440f,     
//  0.961261695938318890f, 0.965925826289068310f, 0.970295726275996470f,     
//  0.974370064785235250f, 0.978147600733805690f,     
//  0.981627183447663980f, 0.984807753012208020f, 0.987688340595137770f,     
//  0.990268068741570360f, 0.992546151641321980f, 0.994521895368273290f,     
//  0.996194698091745550f, 0.997564050259824200f,     
//  0.998629534754573830f, 0.999390827019095760f, 0.999847695156391270f,     
//  1.000000000000000000f, 0.999847695156391270f, 0.999390827019095760f,     
//  0.998629534754573830f, 0.997564050259824200f,     
//  0.996194698091745550f, 0.994521895368273290f, 0.992546151641321980f,     
//  0.990268068741570360f, 0.987688340595137770f, 0.984807753012208020f,     
//  0.981627183447663980f, 0.978147600733805690f,     
//  0.974370064785235250f, 0.970295726275996470f, 0.965925826289068310f,     
//  0.961261695938318890f, 0.956304755963035440f, 0.951056516295153530f,     
//  0.945518575599316850f, 0.939692620785908430f,     
//  0.933580426497201740f, 0.927183854566787420f, 0.920504853452440370f,     
//  0.913545457642600870f, 0.906307787036649940f, 0.898794046299167040f,     
//  0.891006524188367900f, 0.882947592858926990f,     
//  0.874619707139395740f, 0.866025403784438710f, 0.857167300702112330f,     
//  0.848048096156425960f, 0.838670567945424050f, 0.829037572555041620f,     
//  0.819152044288991800f, 0.809016994374947450f,     
//  0.798635510047292830f, 0.788010753606722010f, 0.777145961456970900f,     
//  0.766044443118978010f, 0.754709580222772010f, 0.743144825477394240f,     
//  0.731353701619170570f, 0.719339800338651190f,     
//  0.707106781186547570f, 0.694658370458997370f, 0.681998360062498480f,     
//  0.669130606358858240f, 0.656059028990507280f, 0.642787609686539360f,     
//  0.629320391049837500f, 0.615661475325658290f,     
//  0.601815023152048270f, 0.587785252292473140f, 0.573576436351046050f,     
//  0.559192903470746790f, 0.544639035015027200f, 0.529919264233204900f,     
//  0.515038074910054380f, 0.500000000000000110f,     
//  0.484809620246337110f, 0.469471562785890860f, 0.453990499739546860f,     
//  0.438371146789077460f, 0.422618261740699440f, 0.406736643075800210f,     
//  0.390731128489273940f, 0.374606593415911960f,     
//  0.358367949545300380f, 0.342020143325668820f, 0.325568154457156760f,     
//  0.309016994374947450f, 0.292371704722736770f, 0.275637355816999160f,     
//  0.258819045102520740f, 0.241921895599667900f,     
//  0.224951054343864920f, 0.207911690817759450f, 0.190808995376544920f,     
//  0.173648177666930410f, 0.156434465040230920f, 0.139173100960065690f,     
//  0.121869343405147490f, 0.104528463267653460f,     
//  0.087155742747658138f, 0.069756473744125455f, 0.052335956242943966f,     
//  0.034899496702501080f, 0.017452406437283376f, 0.000000000000000061f,     
//  -0.017452406437283477f, -0.034899496702500733f,     
//  -0.052335956242943620f, -0.069756473744125330f, -0.087155742747658235f,     
//  -0.104528463267653330f, -0.121869343405147370f, -0.139173100960065350f,     
//  -0.156434465040231040f, -0.173648177666930300f,     
//  -0.190808995376544800f, -0.207911690817759120f, -0.224951054343864810f,     
//  -0.241921895599667790f, -0.258819045102520850f, -0.275637355816999050f,     
//  -0.292371704722736660f, -0.309016994374947340f,     
//  -0.325568154457156420f, -0.342020143325668710f, -0.358367949545300270f,     
//  -0.374606593415912070f, -0.390731128489273600f, -0.406736643075800100f,     
//  -0.422618261740699330f, -0.438371146789077510f,     
//  -0.453990499739546750f, -0.469471562785890530f, -0.484809620246337000f,     
//  -0.499999999999999780f, -0.515038074910054270f, -0.529919264233204790f,     
//  -0.544639035015027080f, -0.559192903470746680f,     
//  -0.573576436351045830f, -0.587785252292473030f, -0.601815023152048380f,     
//  -0.615661475325658290f, -0.629320391049837280f, -0.642787609686539360f,     
//  -0.656059028990507500f, -0.669130606358858240f,     
//  -0.681998360062498370f, -0.694658370458997030f, -0.707106781186547460f,     
//  -0.719339800338651300f, -0.731353701619170460f, -0.743144825477394130f,     
//  -0.754709580222772010f, -0.766044443118977900f,     
//  -0.777145961456970680f, -0.788010753606721900f, -0.798635510047292940f,     
//  -0.809016994374947340f, -0.819152044288991580f, -0.829037572555041620f,     
//  -0.838670567945424160f, -0.848048096156425960f,     
//  -0.857167300702112220f, -0.866025403784438710f, -0.874619707139395740f,     
//  -0.882947592858926770f, -0.891006524188367790f, -0.898794046299167040f,     
//  -0.906307787036649940f, -0.913545457642600760f,     
//  -0.920504853452440150f, -0.927183854566787310f, -0.933580426497201740f,     
//  -0.939692620785908320f, -0.945518575599316740f, -0.951056516295153530f,     
//  -0.956304755963035440f, -0.961261695938318670f,     
//  -0.965925826289068200f, -0.970295726275996470f, -0.974370064785235250f,     
//  -0.978147600733805690f, -0.981627183447663980f, -0.984807753012208020f,     
//  -0.987688340595137660f, -0.990268068741570250f,     
//  -0.992546151641321980f, -0.994521895368273290f, -0.996194698091745550f,     
//  -0.997564050259824200f, -0.998629534754573830f, -0.999390827019095760f,     
//  -0.999847695156391270f, -1.000000000000000000f     
//};    
//     
//const float sinTable[360] = {     
//  -0.017452406437283439f, -0.034899496702500699f, -0.052335956242943807f,     
//  -0.069756473744125524f, -0.087155742747658638f, -0.104528463267653730f,     
//  -0.121869343405147550f, -0.139173100960065740f,     
//  -0.156434465040230980f, -0.173648177666930280f, -0.190808995376544970f,     
//  -0.207911690817759310f, -0.224951054343864780f, -0.241921895599667730f,     
//  -0.258819045102521020f, -0.275637355816999660f,     
//  -0.292371704722737050f, -0.309016994374947510f, -0.325568154457156980f,     
//  -0.342020143325668880f, -0.358367949545300210f, -0.374606593415912240f,     
//  -0.390731128489274160f, -0.406736643075800430f,     
//  -0.422618261740699500f, -0.438371146789077290f, -0.453990499739546860f,     
//  -0.469471562785891080f, -0.484809620246337170f, -0.499999999999999940f,     
//  -0.515038074910054380f, -0.529919264233204900f,     
//  -0.544639035015026860f, -0.559192903470746900f, -0.573576436351046380f,     
//  -0.587785252292473250f, -0.601815023152048160f, -0.615661475325658400f,     
//  -0.629320391049837720f, -0.642787609686539470f,     
//  -0.656059028990507280f, -0.669130606358858350f, -0.681998360062498590f,     
//  -0.694658370458997140f, -0.707106781186547570f, -0.719339800338651410f,     
//  -0.731353701619170570f, -0.743144825477394240f,     
//  -0.754709580222771790f, -0.766044443118978010f, -0.777145961456971010f,     
//  -0.788010753606722010f, -0.798635510047292720f, -0.809016994374947450f,     
//  -0.819152044288992020f, -0.829037572555041740f,     
//  -0.838670567945424050f, -0.848048096156426070f, -0.857167300702112330f,     
//  -0.866025403784438710f, -0.874619707139395850f, -0.882947592858927100f,     
//  -0.891006524188367900f, -0.898794046299166930f,     
//  -0.906307787036650050f, -0.913545457642600980f, -0.920504853452440370f,     
//  -0.927183854566787420f, -0.933580426497201740f, -0.939692620785908430f,     
//  -0.945518575599316850f, -0.951056516295153640f,     
//  -0.956304755963035550f, -0.961261695938318890f, -0.965925826289068310f,     
//  -0.970295726275996470f, -0.974370064785235250f, -0.978147600733805690f,     
//  -0.981627183447663980f, -0.984807753012208020f,     
//  -0.987688340595137660f, -0.990268068741570360f, -0.992546151641322090f,     
//  -0.994521895368273400f, -0.996194698091745550f, -0.997564050259824200f,     
//  -0.998629534754573830f, -0.999390827019095760f,     
//  -0.999847695156391270f, -1.000000000000000000f, -0.999847695156391270f,     
//  -0.999390827019095760f, -0.998629534754573830f, -0.997564050259824200f,     
//  -0.996194698091745550f, -0.994521895368273290f,     
//  -0.992546151641321980f, -0.990268068741570250f, -0.987688340595137770f,     
//  -0.984807753012208020f, -0.981627183447663980f, -0.978147600733805580f,     
//  -0.974370064785235250f, -0.970295726275996470f,     
//  -0.965925826289068310f, -0.961261695938318890f, -0.956304755963035440f,     
//  -0.951056516295153530f, -0.945518575599316740f, -0.939692620785908320f,     
//  -0.933580426497201740f, -0.927183854566787420f,     
//  -0.920504853452440260f, -0.913545457642600870f, -0.906307787036649940f,     
//  -0.898794046299167040f, -0.891006524188367790f, -0.882947592858926880f,     
//  -0.874619707139395740f, -0.866025403784438600f,     
//  -0.857167300702112220f, -0.848048096156426070f, -0.838670567945423940f,     
//  -0.829037572555041740f, -0.819152044288991800f, -0.809016994374947450f,     
//  -0.798635510047292830f, -0.788010753606722010f,     
//  -0.777145961456970790f, -0.766044443118978010f, -0.754709580222772010f,     
//  -0.743144825477394240f, -0.731353701619170460f, -0.719339800338651080f,     
//  -0.707106781186547460f, -0.694658370458997250f,     
//  -0.681998360062498480f, -0.669130606358858240f, -0.656059028990507160f,     
//  -0.642787609686539250f, -0.629320391049837390f, -0.615661475325658180f,     
//  -0.601815023152048270f, -0.587785252292473140f,     
//  -0.573576436351046050f, -0.559192903470746900f, -0.544639035015027080f,     
//  -0.529919264233204900f, -0.515038074910054160f, -0.499999999999999940f,     
//  -0.484809620246337060f, -0.469471562785890810f,     
//  -0.453990499739546750f, -0.438371146789077400f, -0.422618261740699440f,     
//  -0.406736643075800150f, -0.390731128489273720f, -0.374606593415912010f,     
//  -0.358367949545300270f, -0.342020143325668710f,     
//  -0.325568154457156640f, -0.309016994374947400f, -0.292371704722736770f,     
//  -0.275637355816999160f, -0.258819045102520740f, -0.241921895599667730f,     
//  -0.224951054343865000f, -0.207911690817759310f,     
//  -0.190808995376544800f, -0.173648177666930330f, -0.156434465040230870f,     
//  -0.139173100960065440f, -0.121869343405147480f, -0.104528463267653460f,     
//  -0.087155742747658166f, -0.069756473744125302f,     
//  -0.052335956242943828f, -0.034899496702500969f, -0.017452406437283512f,     
//  0.000000000000000000f, 0.017452406437283512f, 0.034899496702500969f,     
//  0.052335956242943828f, 0.069756473744125302f,     
//  0.087155742747658166f, 0.104528463267653460f, 0.121869343405147480f,     
//  0.139173100960065440f, 0.156434465040230870f, 0.173648177666930330f,     
//  0.190808995376544800f, 0.207911690817759310f,     
//  0.224951054343865000f, 0.241921895599667730f, 0.258819045102520740f,     
//  0.275637355816999160f, 0.292371704722736770f, 0.309016994374947400f,     
//  0.325568154457156640f, 0.342020143325668710f,     
//  0.358367949545300270f, 0.374606593415912010f, 0.390731128489273720f,     
//  0.406736643075800150f, 0.422618261740699440f, 0.438371146789077400f,     
//  0.453990499739546750f, 0.469471562785890810f,     
//  0.484809620246337060f, 0.499999999999999940f, 0.515038074910054160f,     
//  0.529919264233204900f, 0.544639035015027080f, 0.559192903470746900f,     
//  0.573576436351046050f, 0.587785252292473140f,     
//  0.601815023152048270f, 0.615661475325658180f, 0.629320391049837390f,     
//  0.642787609686539250f, 0.656059028990507160f, 0.669130606358858240f,     
//  0.681998360062498480f, 0.694658370458997250f,     
//  0.707106781186547460f, 0.719339800338651080f, 0.731353701619170460f,     
//  0.743144825477394240f, 0.754709580222772010f, 0.766044443118978010f,     
//  0.777145961456970790f, 0.788010753606722010f,     
//  0.798635510047292830f, 0.809016994374947450f, 0.819152044288991800f,     
//  0.829037572555041740f, 0.838670567945423940f, 0.848048096156426070f,     
//  0.857167300702112220f, 0.866025403784438600f,     
//  0.874619707139395740f, 0.882947592858926880f, 0.891006524188367790f,     
//  0.898794046299167040f, 0.906307787036649940f, 0.913545457642600870f,     
//  0.920504853452440260f, 0.927183854566787420f,     
//  0.933580426497201740f, 0.939692620785908320f, 0.945518575599316740f,     
//  0.951056516295153530f, 0.956304755963035440f, 0.961261695938318890f,     
//  0.965925826289068310f, 0.970295726275996470f,     
//  0.974370064785235250f, 0.978147600733805580f, 0.981627183447663980f,     
//  0.984807753012208020f, 0.987688340595137770f, 0.990268068741570250f,     
//  0.992546151641321980f, 0.994521895368273290f,     
//  0.996194698091745550f, 0.997564050259824200f, 0.998629534754573830f,     
//  0.999390827019095760f, 0.999847695156391270f, 1.000000000000000000f,     
//  0.999847695156391270f, 0.999390827019095760f,     
//  0.998629534754573830f, 0.997564050259824200f, 0.996194698091745550f,     
//  0.994521895368273400f, 0.992546151641322090f, 0.990268068741570360f,     
//  0.987688340595137660f, 0.984807753012208020f,     
//  0.981627183447663980f, 0.978147600733805690f, 0.974370064785235250f,     
//  0.970295726275996470f, 0.965925826289068310f, 0.961261695938318890f,     
//  0.956304755963035550f, 0.951056516295153640f,     
//  0.945518575599316850f, 0.939692620785908430f, 0.933580426497201740f,     
//  0.927183854566787420f, 0.920504853452440370f, 0.913545457642600980f,     
//  0.906307787036650050f, 0.898794046299166930f,     
//  0.891006524188367900f, 0.882947592858927100f, 0.874619707139395850f,     
//  0.866025403784438710f, 0.857167300702112330f, 0.848048096156426070f,     
//  0.838670567945424050f, 0.829037572555041740f,     
//  0.819152044288992020f, 0.809016994374947450f, 0.798635510047292720f,     
//  0.788010753606722010f, 0.777145961456971010f, 0.766044443118978010f,     
//  0.754709580222771790f, 0.743144825477394240f,     
//  0.731353701619170570f, 0.719339800338651410f, 0.707106781186547570f,     
//  0.694658370458997140f, 0.681998360062498590f, 0.669130606358858350f,     
//  0.656059028990507280f, 0.642787609686539470f,     
//  0.629320391049837720f, 0.615661475325658400f, 0.601815023152048160f,     
//  0.587785252292473250f, 0.573576436351046380f, 0.559192903470746900f,     
//  0.544639035015026860f, 0.529919264233204900f,     
//  0.515038074910054380f, 0.499999999999999940f, 0.484809620246337170f,     
//  0.469471562785891080f, 0.453990499739546860f, 0.438371146789077290f,     
//  0.422618261740699500f, 0.406736643075800430f,     
//  0.390731128489274160f, 0.374606593415912240f, 0.358367949545300210f,     
//  0.342020143325668880f, 0.325568154457156980f, 0.309016994374947510f,     
//  0.292371704722737050f, 0.275637355816999660f,     
//  0.258819045102521020f, 0.241921895599667730f, 0.224951054343864780f,     
//  0.207911690817759310f, 0.190808995376544970f, 0.173648177666930280f,     
//  0.156434465040230980f, 0.139173100960065740f,     
//  0.121869343405147550f, 0.104528463267653730f, 0.087155742747658638f,     
//  0.069756473744125524f, 0.052335956242943807f, 0.034899496702500699f,     
//  0.017452406437283439f, 0.000000000000000122f     
//};     

// Calculate Poisson Probability
float myMath_poissonProb(int lamda, int k)
{
	float sum = 0.f;
	int i = 1;
	float ln_P = 0.f;
	for (; i <= k; i++) {
		sum = sum + logf((float)(i));
	}
	ln_P = -1.0f * (float)(lamda) + (float)(k)*((float)logf((float)(lamda))) - sum;
	return (float)exp(ln_P);
}

// Calculate atan2f, atan(xnumerator/denominator),-pi~pi
float myMath_atan2f(float numerator, float denominator)
{
	float azim = 0.f;
	float range = sqrtf(numerator*numerator+denominator*denominator);
	myMath_checkZero(&range);
	/* calc azimuth */
	if (denominator == 0)
	{
		if(numerator>0)
		{
			azim = M_PI/2.0f;
		}
		else if(numerator<0)
		{
			azim = -1.f*M_PI/2.0f;
		}
		else
		{
			azim = 0.f;
		}
	}
	else if (denominator > 0)
		//azim = atanf(numerator/denominator);
		azim = asinf(numerator/range);
	else  //(denominator < 0)
	{
		if (numerator > 0)
			azim = M_PI - asinf(numerator / range);
		else if (numerator < 0)
			azim = -M_PI - asinf(numerator / range);
		else
			azim = M_PI;
	}
	return azim;
}


// convert (range, azimuth, doppler, elevation) -> (x,y,z,vx,vy,vz)
int myMath_spherical2cartesian(float *sph, float *cart)
{
	float range; 
	float azim; 
	float pitchRad; 
	float doppler;
    float sinAzim, cosAzim;
    float sinElev, cosElev;
#ifndef MMWAVE_3D
	range = sph[0];
	azim = sph[1];
	doppler = sph[2];
	myMath_sincosd(azim*RAD2DEG, &sinAzim, &cosAzim);
	cart[0] = range * sinAzim; //((float)(sin((double)azim)));
	cart[1] = range * cosAzim; //((float)(cos((double)azim)));
	cart[2] = doppler * sinAzim; //((float)(sin((double)azim)));
	cart[3] = doppler * cosAzim; //((float)(cos((double)azim)));
#else
	range = sph[0];
	azim = sph[1];
	doppler = sph[2];
	pitchRad = sph[3];
	myMath_sincosd(azim*RAD2DEG, &sinAzim, &cosAzim);
	myMath_sincosd(pitchRad*RAD2DEG, &sinElev, &cosElev);
	cart[0] = range * cosElev * sinAzim; //((float)(cos((double)pitchRad)))*((float)(sin((double)azim)));
	cart[1] = range * cosElev * cosAzim; //((float)(cos((double)pitchRad)))*((float)(cos((double)azim)));
	cart[2] = range * sinElev; //((float)(sin((double)pitchRad)));
	cart[3] = doppler * cosElev * sinAzim; //((float)(cos((double)pitchRad)))*((float)(sin((double)azim)));
	cart[4] = doppler * cosElev * cosAzim; //((float)(cos((double)pitchRad)))*((float)(cos((double)azim)));
	cart[5] = doppler * sinElev; //((float)(sin((double)pitchRad)));
#endif
	return 0;
}

// convert (x,y,z,vx,vy,vz) -> (range, azimuth, doppler, elevation)
int myMath_cartesian2spherical(float *cart, float *sph)
{
	float posx, posy, posz; 
	float velx, vely, velz;
	float range;

#ifndef MMWAVE_3D
	posx = cart[0]; posy = cart[1]; 
	velx = cart[2]; vely = cart[3];
	/* calc range */
	sph[0] = sqrtf(posx*posx + posy*posy); 
	if (sph[0] == 0) {
		return -1;
	}
	/* calc azimuth */
	sph[1] = myMath_atan2f(posx, posy);
	/* calc doppler */
	range = sph[0];
	myMath_checkZero(&range);
	sph[2] = (posx*velx+posy*vely)/range;	
#else
	/* cart = [posx posy posz velx vely velz] */
	/* sph = [range azimuth doppler elevation] */
	posx = cart[0]; posy = cart[1]; posz = cart[2];
	velx = cart[3]; vely = cart[4]; velz = cart[5];
	/* calc range */
	sph[0] = sqrtf(posx*posx + posy*posy + posz*posz); 
	if (sph[0] == 0) {
		return -1;
	}
	/* calc azimuth */
	sph[1] = myMath_atan2f(posy, posx);
	/* calc doppler */
	range = sph[0];
	myMath_checkZero(&range);
	sph[2] = (posx*velx+posy*vely+posz*velz)/range;	
	if (posx*posx + posy * posy == 0) {
		return -1;
	}
	/* calc elevation */
	sph[3] = myMath_atan2f(posz, sqrtf(posx*posx+posy*posy));
#endif
	return 0;
}

/**
*  @b Description
*  @n
*		This function is used to compute sine and cosine of the angle in degrees
*  @param[in]  theta
*		Angle, in degrees
*		Matrix B
*  @param[out] pSinVal
*		pointer to computed sine value, degrees
*  @param[out] pCosVal
*		pointer to computed sine value, degrees
*  @retval
*      None
*/
void myMath_sincosd(float theta, float * pSinVal, float * pCosVal)     
{     
    float x1 = -179.0f;                        /* Initial input value */     
    float ftheta = 0.f;  
	
	*pSinVal = sinf(theta*DEG2RAD);
	*pCosVal = cosf(theta*DEG2RAD);
	return;

//	myMath_unrolling(&theta, -179.f, 180.f);
//    ftheta = (float)((int32_t)theta);  

//    i = (uint32_t) (fabsf(theta - x1));     

//    /* Calculation of fractional part */     
//    if(theta >= 0.0f)     
//    {     
//        fract = theta - ftheta;     
//    }     
//    else     
//    {     
//        fract = -1.0f * ((theta - ftheta) + 1.0f);     
//    }     
//    /* reading nearest sine output values */     
//    ysin0 = sinTable[i];     
//    ysin1 = sinTable[i + 1u];     

//    /* reading nearest cosine output values */     
//    ycos0 = cosTable[i];     
//    ycos1 = cosTable[i + 1u];     

//    /* difference of nearest sine output value */  
//    ysin1 = ysin1 - ysin0;  

//    /* difference of nearest cosine output value */  
//    ycos1 = ycos1 - ycos0;  

//    /* Calculation of sine value */     
//    *pSinVal = ysin0 + (fract * ysin1);

//    /* Calculation of cosine value */     
//    *pCosVal = ycos0 + (fract * ycos1);
}     

// convert log2(SNR) to db
float convert_Log2SNR_to_db(int32_t inputActual, uint16_t fracBitIn)
{
    float output;
    float input = (float)(inputActual) / ((float)(1 << fracBitIn));
    output = input * 6.0f; //From_Log2_to_20Log10
    return output;
}

// Descending order in float
int cmp_descending_float(const void *a, const void *b)
{
	return ((*(float*)b - *(float*)a > 0) ? 1 : -1);
}

// Ascending order in float
int cmp_Ascending_float(const void *a, const void *b)
{
	return ((*(float*)a - *(float*)b > 0) ? 1 : -1);
}

// Descending order in int
int cmp_descending_int(const void *a, const void *b)
{
	return ((*(int*)b - *(int*)a > 0) ? 1 : -1);
}

// Ascending order in int
int cmp_Ascending_int(const void *a, const void *b)
{
	return ((*(int*)a - *(int*)b > 0) ? 1 : -1);
}

void myMath_checkZero(float *val)
{
	if(fabsf(*val)<0.0001f)
	{
		if(*val<0.f)
		{
			*val = -0.0001f;
		}
		else
		{
			*val = 0.0001f;
		}
	}
}

void myMath_nomalization(float *valArray, int k)
{
	int i = 0;
	float valSum = 0.f;
	for(i=0; i<k; i++)
	{
		valSum += fabsf(valArray[i]);
	}
	for(i=0; i<k; i++)
	{
		valArray[i] = (valSum==0)? 0.f:valArray[i]/valSum;
	}
}

//lowPassFilter, x = k*input + (1-k)*x
void myMath_lowPassFilter(float *x, float input, float k)
{
	float x_ = *x;
	*x = k*input + (1.0f-k)*x_;
}

// Set a value into [min_, max_]
void myMath_minMax(float *x, float min_, float max_)
{
	if((*x)<min_)
	{
		*x = min_;
	}
	else if((*x)>max_)
	{
		*x = max_;
	}
}

// get max value
float myMath_getMax(float a, float b)
{
	if(a>b){
		return a;
	}
	else{
		return b;
	}
}

// get min value
float myMath_getMin(float a, float b)
{
	if(a>b){
		return b;
	}
	else{
		return a;
	}
}


// unroll a value between min_ and max_ 
void myMath_unrolling(float *x, float min_, float max_)
{
	float xOut = 0.f;
	float xIn = *x;
	float dist = max_ - min_;//(90-1)-(-90+1)=178
	myMath_checkZero(&dist);
	if(xIn>=min_ && xIn<=max_)
	{
		xOut = xIn;
	}
	else if(xIn>max_)
	{
		//磺xIn=90,xOut= ((90-89)/178-|(90-89)/178|)*178-89 = -88
		xOut = ( (xIn-max_)/dist - ((float)((int)((xIn-max_)/dist))) ) * (max_-min_) - max_;
	}
	else if(xIn<min_)
	{
		//磺xIn= -90,xOut= ((-90-(-89))/178-|(-90-(-89))/178|)*178+89 = 88
		xOut = ( (xIn-min_)/dist - ((float)((int)((xIn-min_)/dist))) ) * (max_-min_) + max_;
	}
	*x = xOut;
}

// Line interpolate a value in [min_, max_] based on the thresholds [thres_min, thres_max]
//For example (x<0.5 -> x=0.02; x>5 -> x=1; x belongs to [0.5. 5], lineInterpolate between 0.02 and 1)
void myMath_lineInterpolation(float *x, float min_, float max_, float thres, float thres_min, float thres_max)
{
	float xIn = *x;
	float xOut = *x;
	float diff_thres;
	if(thres<=thres_min)
	{
		xOut = min_;
	}
	else if(thres>=thres_max)
	{
		xOut = max_;
	}
	else
	{
		diff_thres = thres_max-thres_min;
		myMath_checkZero(&diff_thres);
		xOut = (thres-thres_min)/diff_thres * (max_-min_) + min_;
	}
	*x = xOut;
}

uint8_t myMath_isPointInsideBox(float *c, sBox *box)
{
    if( (c[0] > box->x1) && (c[0] < box->x2) &&
        (c[1] > box->y1) && (c[1] < box->y2) )
        return 1U;
    else
        return 0;
}

void myMath_insertSort(sDetectPoint* allMeaDataBuff, int num)
{
	int i, j;
	int first = 0;
	sDetectPoint mea_tmp;
	for (i = 1; i < num; i++) 
	{
		memcpy(&mea_tmp, &allMeaDataBuff[i], sizeof(allMeaDataBuff[i]));
		j = i - 1;
		while (j >= first && allMeaDataBuff[j].range > mea_tmp.range) 
		{
			memcpy(&allMeaDataBuff[j+1], &allMeaDataBuff[j], sizeof(allMeaDataBuff[j]));
			j--;
		}
		memcpy(&allMeaDataBuff[j+1], &mea_tmp, sizeof(allMeaDataBuff[j+1]));
	}
}

void myMath_insertSort_float(float* allMeaDataBuff, int num)
{
	int i, j;
	int first = 0;
	float mea_tmp;
	for (i = 1; i < num; i++) 
	{
		memcpy(&mea_tmp, &allMeaDataBuff[i], sizeof(allMeaDataBuff[i]));
		j = i - 1;
		while (j >= first && allMeaDataBuff[j] > mea_tmp) 
		{
			memcpy(&allMeaDataBuff[j+1], &allMeaDataBuff[j], sizeof(allMeaDataBuff[j]));
			j--;
		}
		memcpy(&allMeaDataBuff[j+1], &mea_tmp, sizeof(allMeaDataBuff[j+1]));
	}
}

void myMath_swap(float *a, float *b)
{
	float c = *a;
	*a = *b;
	*b = c;
}

void myMath_swap_int16(int16_t *a, int16_t *b)
{
	int16_t c = *a;
	*a = *b;
	*b = c;
}

// Generate the dummy data for radar, covering [-4, 5] in lateral and [0, 150] in longitudinal
int myMath_generateDummyRadarData(sDummyRadarMeasUnit* dummyData, int* num, float vx, float vy, float vz,uint8_t enable_stat)
{
	int nd = 0;
	if(dummyData==NULL)
	{
		return -1;
	}

	nd = *num;
	if(dummy_frameCnt==0)
	{
		memset(dummyData, 0, sizeof(sDummyRadarMeasUnit)*(nd*(PT_NUM_ADD_TO_GATHER+1)));
		myMath_initDummyRadarData(dummyData, num, vx, vy, vz);
	}
	else
	{
		myMath_getNextFrameDummyRadarData(dummyData, num, vx, vy, vz, enable_stat);
	}
	*num = nd*(PT_NUM_ADD_TO_GATHER+1);
	dummy_frameCnt++;
	return 0;
}

void myMath_initDummyRadarData(sDummyRadarMeasUnit* dummyData, int* num, float vx, float vy,float vz)
{
	int rows_ = 0;
	int cols_ = 0;
	int nd = *num;
	float col_vals[10] = {0.f*DUMMY_SCALE, -1.f*DUMMY_SCALE, 1.f*DUMMY_SCALE, -2.f*DUMMY_SCALE,\
		2.f*DUMMY_SCALE, -3.f*DUMMY_SCALE, 3.f*DUMMY_SCALE, -4.f*DUMMY_SCALE, 4.f*DUMMY_SCALE, 5.f*DUMMY_SCALE};
	int i = 0;
	int j = 0;
	int k = 0;
	float fLongResolution = 0.f;
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;
	float velx = 0.f;
	float vely = 0.f;
	float velz = 0.f;
	float theta = 0.f;
	float sph[4];
	float targetState[9] = { 0 };
	int pt_num_add_to_gather=0;
	cols_ = 10;
	if (nd == 1)
	{
		cols_ = 1;
	}
	rows_ = (int)(nd/cols_);
	if(cols_*rows_<nd)
	{
		rows_ += 1;
	}
	fLongResolution = 300.f/((float)(rows_));
	if(nd==2)
	{
		rows_ = 2;
		cols_ = 1;
		fLongResolution = 50.f;
	}
	for(i=0; i<rows_; i++)
	{
		for(j=0; j<cols_; j++)
		{
			x = fLongResolution * ((float)(i)) + 600.f;
			y = col_vals[j];
			velx = vx;
			vely = vy;
			velz = vz;
			dummyData[k].uStatFlag = 0U;
			if(nd==2)
			{
				x = fLongResolution * ((float)(i)) + 2.f;
				y = col_vals[j];
				if((j+1)%2==0)
				{
					velx = 0;
					vely = 0;
					velz = 0;
					dummyData[k].uStatFlag = 1U;
				}
				else
				{
					velx = vx;
					vely = vy;
					velz = vz;
					dummyData[k].uStatFlag = 0U;
				}
			}
			z = 100.f;
			targetState[0] = x; targetState[1] = y; targetState[2] = z;
			targetState[3] = velx; targetState[4] = vely; targetState[5] = velz;
			myMath_cartesian2spherical(targetState, sph);
			dummyData[k].x = x;
			dummyData[k].y = y;
			dummyData[k].z = z;
			dummyData[k].vx = velx;
			dummyData[k].vy = vely;
			dummyData[k].vz = velz;
			dummyData[k].range = sph[0];
			dummyData[k].azimuth= sph[1] * RAD2DEG;
			dummyData[k].doppler = sph[2];
			dummyData[k].pitch = sph[3] * RAD2DEG;

			//////////////////////////////////////////////////////////////////////////
			pt_num_add_to_gather=0;
			while (PT_NUM_ADD_TO_GATHER > pt_num_add_to_gather)
			{
				int index=nd*(1+pt_num_add_to_gather)+k;
				dummyData[index].x = x+ 2*pt_num_add_to_gather;// rand() % DATA_RAND_MAX;
				dummyData[index].y = y+ 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX;
				dummyData[index].z = z+ 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX;
				dummyData[index].vx = velx+ 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.1;
				dummyData[index].vy = vely+ 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.1;
				dummyData[index].vz = velz+ 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.1;
				dummyData[index].range = sph[0] + 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX;
				dummyData[index].azimuth= sph[1] * RAD2DEG+rand() % DATA_RAND_MAX*0.01;
				dummyData[index].doppler = sph[2]+ 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.1;
				dummyData[index].pitch = sph[3] * RAD2DEG+rand() % DATA_RAND_MAX*0.01;

				pt_num_add_to_gather++;
			}

			k++;
		}
	}
	*num = k;
}

void myMath_getNextFrameDummyRadarData(sDummyRadarMeasUnit* dummyData, int* num, float vx, float vy,float vz, uint8_t enable_stat)
{
	float x = 0.f;
	float y = 0.f;
	float z = 0.f;
	float velx = 0.f;
	float vely = 0.f;
	float velz = 0.f;
	float theta = 0.f;
	float sph[4];
	float targetState[9] = { 0 };
	int nd = *num;
	int pt_num_add_to_gather=0;
	int i = 0;
	for(i=0; i<*num; i++)
	{
		velx = vx;
		vely = vy;
		velz = vz;
#ifdef CONST_DELTA_T
		x = dummyData[i].x + CONST_DELTA_T*vx;
#else
		x = dummyData[i].x + 0.1f*vx;
#endif
		if (x < 0.f) {
			x = 800.f;
		}
		
#ifdef CONST_DELTA_T
		y = dummyData[i].y + CONST_DELTA_T*vy;
#else
		y = dummyData[i].y + 0.1f*vy;
#endif
		if (y > 5.f*DUMMY_SCALE) {
			y = -4.f*DUMMY_SCALE;
		}
		else if (y < -4.f*DUMMY_SCALE) {
			y = 5.f*DUMMY_SCALE;
		}
		if((*num)==2)
		{
			if(y>300.f){
				y = -300.f;
			}
			else if(y<-300.f){
				y = 300.f;
			}
		}
#ifdef CONST_DELTA_T
		z = dummyData[i].z + CONST_DELTA_T * vz;
#else
		z = dummyData[i].z + 0.1f*vz;
#endif

		if (z < 0.f) {
			z = 100;
		}
		targetState[0] = x; targetState[1] = y; targetState[2] = z;
		targetState[3] = velx; targetState[4] = vely; targetState[5] = velz;
		myMath_cartesian2spherical(targetState, sph);
		dummyData[i].x = x;
		dummyData[i].y = y;
		dummyData[i].z = z;
		dummyData[i].vx = vx;
		dummyData[i].vy = vy;
		dummyData[i].vz = vz;
		dummyData[i].range = sph[0];
		dummyData[i].azimuth = sph[1] * RAD2DEG;
		dummyData[i].doppler = sph[2];
		dummyData[i].pitch = sph[3] * RAD2DEG;

		//////////////////////////////////////////////////////////////////////////
		pt_num_add_to_gather=0;
		while (PT_NUM_ADD_TO_GATHER > pt_num_add_to_gather)
		{
			int index=nd*(1+pt_num_add_to_gather)+i;
			dummyData[index].x = x + 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX;
			dummyData[index].y = y + 1 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX;
			dummyData[index].z = z + 1 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX;
			dummyData[index].vx = velx + 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.1;
			dummyData[index].vy = vely + 1 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.1;
			dummyData[index].vz = velz + 1 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.1;
			dummyData[index].range = sph[0] - 1 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX;
			dummyData[index].azimuth= sph[1] * RAD2DEG + 1 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.01;
			dummyData[index].doppler = sph[2] + 1 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.1;
			dummyData[index].pitch = sph[3] * RAD2DEG + 2 * pt_num_add_to_gather;// rand() % DATA_RAND_MAX*0.01;

			pt_num_add_to_gather++;
		}
	}
}

// Read the dummy data for radar
int myMath_readDummyRadarData(sDummyRadarMeasUnit* dummyData, int* num, int frameID)
{
	int nd = 0;
	int pt_num_add_to_gather=0;
	int bInFlag=0;
	int k=0;

	if(dummyData==NULL)
		return -1;

	nd = *num;
	//if(dummy_frameCnt==0)
	memset(dummyData, 0, sizeof(sDummyRadarMeasUnit)*(nd*(PT_NUM_ADD_TO_GATHER+1)));
	//myMath_initDummyRadarData(dummyData, num, vx, vy, vz);

	for (k=0;k<MAX_STEPS_NUM;k++)
		if (g_arr_nTimeSeq[k]==frameID)
			bInFlag=1;

	if (!bInFlag){
		*num=0;
	} 
	else{//x,vx,y,vy,z,vz,  range,azimuth,elevation,doppler
		//CONST_DELTA_T
		int index=0;
		int indexDummyData=g_nDummyDataUsedIndex*10;
		dummyData[index].x = g_arr_fDummyData[indexDummyData+0];
		dummyData[index].y = g_arr_fDummyData[indexDummyData+2];
		dummyData[index].z = g_arr_fDummyData[indexDummyData+4];
		dummyData[index].vx = g_arr_fDummyData[indexDummyData+1];
		dummyData[index].vy = g_arr_fDummyData[indexDummyData+3];
		dummyData[index].vz = g_arr_fDummyData[indexDummyData+5];
		dummyData[index].range = g_arr_fDummyData[indexDummyData+6];
		dummyData[index].azimuth = g_arr_fDummyData[indexDummyData+7];
		dummyData[index].doppler = g_arr_fDummyData[indexDummyData+9];
		dummyData[index].pitch = g_arr_fDummyData[indexDummyData+8];

		//////////////////////////////////////////////////////////////////////////
		pt_num_add_to_gather=0;
		while ((PT_NUM_ADD_TO_GATHER+nd-1) > pt_num_add_to_gather){
			index++;
			dummyData[index].x = dummyData[0].x+rand() % DATA_RAND_MAX;
			dummyData[index].y = dummyData[0].y+rand() % DATA_RAND_MAX;
			dummyData[index].z = dummyData[0].z+rand() % DATA_RAND_MAX;
			dummyData[index].vx = dummyData[0].vx+rand() % DATA_RAND_MAX*0.1;
			dummyData[index].vy = dummyData[0].vx+rand() % DATA_RAND_MAX*0.1;
			dummyData[index].vz = dummyData[0].vx+rand() % DATA_RAND_MAX*0.1;
			dummyData[index].range = dummyData[0].range+rand() % DATA_RAND_MAX;
			dummyData[index].azimuth= dummyData[0].azimuth+rand() % DATA_RAND_MAX*0.01;
			dummyData[index].doppler = dummyData[0].doppler+rand() % DATA_RAND_MAX*0.1;
			dummyData[index].pitch = dummyData[0].pitch+rand() % DATA_RAND_MAX*0.01;

			pt_num_add_to_gather++;
		}

		*num = nd + PT_NUM_ADD_TO_GATHER;
		g_nDummyDataUsedIndex++;
	}

	dummy_frameCnt++;
	return 0;
}

// Generate the dummy data for radar, covering [-4, 5] in lateral and [0, 150] in longitudinal
int myMath_generateDummyRadarData_tas1(sDummyRadarMeasUnit* dummyData, int* num)
{
	int i;
	int nd = 0;
	int nNum=0;
	int nLoopNum;
	if(dummyData==NULL)
	{
		return -1;
	}

	nd = *num;
	nLoopNum=nd*(PT_NUM_ADD_TO_GATHER+1);
	if(dummy_frameCnt==0)
	{
		memset(dummyData, 0, sizeof(sDummyRadarMeasUnit)*nLoopNum);
		//myMath_initDummyRadarData(dummyData, num);
		for (i = 0; i < nLoopNum; i++)
		{
			//int aziBeamId=rand()%20;
			//int eleBeamId=rand()%4;
			int aziBeamId=14;
			int eleBeamId=0;
			float azimuth=(-58+4*aziBeamId)*DEG2RAD;
			float elevataion=(-15+10*eleBeamId)*DEG2RAD;

			int index=(aziBeamId*4+eleBeamId)+i*10;
			//arr_nTimeSeq[i]=index;

			//float fRange=20+rand()%2000;
			//float fV=-100+rand()%20;//[-10,10]
			//float angle1=0;//-90+rand()%180
			//float angle2=-90+rand()%180;

			float fRange=400;
			//float fV=-100+rand()%20;//[-10,10]
			//float angle1=0*DEG2RAD;//-90+rand()%180
			//float angle2=-60*DEG2RAD;

			dummyData[i].x=fRange*cos(elevataion)*cos(azimuth);
			dummyData[i].y=fRange*cos(elevataion)*sin(azimuth);
			dummyData[i].z=fRange*sin(elevataion);

			//dummyData[i].vx=fV*sin(angle1);
			//dummyData[i].vy=fV*cos(angle1)*sin(angle2);
			//dummyData[i].vz=fV*cos(angle1)*cos(angle2);
			dummyData[i].vx=0;
			dummyData[i].vy=20;
			dummyData[i].vz=0;

			dummyData[i].range=fRange;
			dummyData[i].azimuth=azimuth* RAD2DEG;
			dummyData[i].pitch=elevataion* RAD2DEG;
			dummyData[i].doppler=(dummyData[i].x*dummyData[i].vx+dummyData[i].y*dummyData[i].vy+dummyData[i].z*dummyData[i].vz)/dummyData[i].range;
		}
	}
	else
	{
		//myMath_getNextFrameDummyRadarData(dummyData, num);
		for (i = 0; i < nLoopNum; i++)
		{
#ifdef CONST_DELTA_T
			float x=dummyData[i].x+dummyData[i].vx*CONST_DELTA_T;
			float y=dummyData[i].y+dummyData[i].vy*CONST_DELTA_T;
			float z=dummyData[i].z+dummyData[i].vz*CONST_DELTA_T;
#else
			float x = dummyData[i].x + dummyData[i].vx / 120.f;
			float y = dummyData[i].y + dummyData[i].vy / 120.f;
			float z = dummyData[i].z + dummyData[i].vz / 120.f;
#endif

			dummyData[i].x=x;
			dummyData[i].y=y;
			dummyData[i].z=z;
			dummyData[i].range=sqrtf(x*x+y*y+z*z);
			dummyData[i].azimuth=atan2(y,x)* RAD2DEG;
			dummyData[i].pitch=atan2(z,sqrtf(x*x+y*y))* RAD2DEG;
			dummyData[i].doppler=(x*dummyData[i].vx+y*dummyData[i].vy+z*dummyData[i].vz)/dummyData[i].range;

			//if (-60< dummyData[i].azimuth && dummyData[i].azimuth < 60 && -20 < dummyData[i].pitch && dummyData[i].pitch < 20){
			//	nNum++; 
			//}
				
		}
	}

	*num = nLoopNum;//nd*(PT_NUM_ADD_TO_GATHER+1);
	dummy_frameCnt++;
	return 0;
}

int myMath_generateDummyRadarData_tas(sDummyRadarMeasUnit* dummyData, int* num)
{
	int i;
	int nd = 0;
	int nNum=0;
	int nLoopNum;
	if(dummyData==NULL)
	{
		return -1;
	}

	nd = *num;
	nLoopNum=nd*(PT_NUM_ADD_TO_GATHER+1);
	if(dummy_frameCnt==0)
	{
		memset(dummyData, 0, sizeof(sDummyRadarMeasUnit)*nLoopNum);
		//myMath_initDummyRadarData(dummyData, num);
		for (i = 0; i < nLoopNum; i++)
		{
			int aziBeamId=rand()%20;
			int eleBeamId=rand()%4;
			float azimuth=(-58+4*aziBeamId)*DEG2RAD;
			float elevataion=(-15+10*eleBeamId)*DEG2RAD;

			//int index=(aziBeamId*4+eleBeamId)+i*10;
			//arr_nTimeSeq[i]=index;

			float fRange=20+rand()%1000;
			float fV=-10+rand()%20;//[-10,10]
			float angle1=(-90+rand()%180)*DEG2RAD;//-90+rand()%180
			float angle2=(-30+rand()%60)*DEG2RAD;

			dummyData[i].x=fRange*cos(elevataion)*cos(azimuth);
			dummyData[i].y=fRange*cos(elevataion)*sin(azimuth);
			dummyData[i].z=fRange*sin(elevataion);

			dummyData[i].vx=fV*sin(angle1);
			dummyData[i].vy=fV*cos(angle1)*sin(angle2);
			dummyData[i].vz=fV*cos(angle1)*cos(angle2);

			dummyData[i].range=fRange;
			dummyData[i].azimuth=azimuth* RAD2DEG;
			dummyData[i].pitch=elevataion* RAD2DEG;
			dummyData[i].doppler=(dummyData[i].x*dummyData[i].vx+dummyData[i].y*dummyData[i].vy+dummyData[i].z*dummyData[i].vz)/dummyData[i].range;
		}
	}
	else
	{
		//myMath_getNextFrameDummyRadarData(dummyData, num);
		for (i = 0; i < nLoopNum; i++)
		{
#ifdef CONST_DELTA_T
			float x=dummyData[i].x+dummyData[i].vx*CONST_DELTA_T;
			float y=dummyData[i].y+dummyData[i].vy*CONST_DELTA_T;
			float z=dummyData[i].z+dummyData[i].vz*CONST_DELTA_T;
#else
			float x = dummyData[i].x + dummyData[i].vx / 120.f;
			float y = dummyData[i].y + dummyData[i].vy / 120.f;
			float z = dummyData[i].z + dummyData[i].vz / 120.f;
#endif
			dummyData[i].x=x;
			dummyData[i].y=y;
			dummyData[i].z=z;
			dummyData[i].range=sqrtf(x*x+y*y+z*z);
			dummyData[i].azimuth=atan2(y,x)* RAD2DEG;
			dummyData[i].pitch=atan2(z,sqrtf(x*x+y*y))* RAD2DEG;
			dummyData[i].doppler=(x*dummyData[i].vx+y*dummyData[i].vy+z*dummyData[i].vz)/dummyData[i].range;

			//if (-60< dummyData[i].azimuth && dummyData[i].azimuth < 60 && -20 < dummyData[i].pitch && dummyData[i].pitch < 20){
			//	nNum++; 
			//}

		}
	}

	*num = nLoopNum;//nd*(PT_NUM_ADD_TO_GATHER+1);
	dummy_frameCnt++;
	return 0;
}

// from meter to pixel
int convert_y(double y)
{
	return (int)(((double)(IMG_WIDTH))/2.0 - y*((double)(IMG_WIDTH_SCALE)));
}

// from meter to pixel
int convert_x(double x)
{
	return (int)((double)(IMG_HEIGHT)-((double)(IMG_HEIGHT)) / HEIGHT_SPLIT_SCALE - x*((double)(IMG_HEIGHT_SCALE)));
}

// from pixel to meter
double inv_convert_y(int y)
{
	return ((((double)(IMG_WIDTH)) / 2.0 - ((double)(y))) / ((double)(IMG_WIDTH_SCALE)));
}

// from pixel to meter
double inv_convert_x(int x)
{
	//return (((double)(IMG_HEIGHT - x)) - ((double)(IMG_HEIGHT)) ) / ((double)(IMG_HEIGHT_SCALE));
	return ((double)(IMG_HEIGHT-((double)(IMG_HEIGHT)) / HEIGHT_SPLIT_SCALE) - x) / ((double)(IMG_HEIGHT_SCALE));
}

#ifdef DEBUG_LOG
void __cdecl my_printf(const char *strOutputString, ...)
{
    char strBuffer[4096] = {0};
	int len = 0;
    va_list vlArgs;
    va_start(vlArgs, strOutputString);
    _vsnprintf_s(strBuffer, sizeof(strBuffer) - 1, 4096, _T(strOutputString), vlArgs);
    va_end(vlArgs);
    OutputDebugString(strBuffer);

	if(logOut_fp==NULL && g_logOutPath!=NULL){
		if((logOut_fp=fopen(g_logOutPath, "w"))==NULL){
			g_logOutPath = NULL;
			logOut_fp = NULL;
			return;
		}
	}
	if(logOut_fp!=NULL){
		fprintf(logOut_fp, "%s", strBuffer);
	}

	len = strlen(strBuffer);
	//while(1==log_mutex || _log_ptr>=(MAX_SIZE_LOG_STR-len-1)){
	while(1==log_mutex || _log_ptr>0){
		Sleep(0.5);
	}
	log_mutex = 1;
	if(_log_ptr<MAX_SIZE_LOG_STR){
		memcpy(&_log_str[_log_ptr], strBuffer, sizeof(char)*len);
		_log_ptr += len;
	}
	log_mutex = 0;
}
#endif

#ifdef ENABLE_OUTRESULT
void my_setOutPath(char* outPath, char* logOutPath)
{
	g_outPath = outPath;
	g_logOutPath = logOutPath;
}

void my_closeOutFile()
{
	if(out_fp!=NULL && g_outPath!=NULL){
		fclose(out_fp);
		g_outPath = NULL;
		out_fp = NULL;
	}
	if(warningOut_fp!=NULL && g_warningOutPath!=NULL){
		fclose(warningOut_fp);
		g_warningOutPath = NULL;
		warningOut_fp = NULL;
	}
	if(logOut_fp!=NULL && g_logOutPath!=NULL){
		fclose(logOut_fp);
		g_logOutPath = NULL;
		logOut_fp = NULL;
	}
}

void __cdecl my_out2File(const char *strOutputString, ...)
{
    char strBuffer[4096] = {0};
	int len = 0;
    va_list vlArgs;

	if(out_fp==NULL && g_outPath!=NULL){
		if((out_fp=fopen(g_outPath, "w"))==NULL){
#ifdef DEBUG_LOG
			//my_printf("Fail to open: %s \n", g_outPath);
			g_outPath = NULL;
			out_fp = NULL;
			return;
#endif
		}
	}
    va_start(vlArgs, strOutputString);
    _vsnprintf_s(strBuffer, sizeof(strBuffer) - 1, 4096, _T(strOutputString), vlArgs);
	fprintf(out_fp, "%s", strBuffer);
	fflush(out_fp);
}

void __cdecl my_out2WarningFile(const char *strOutputString, ...)
{
    char strBuffer[4096] = {0};
	int len = 0;
    va_list vlArgs;

	if(warningOut_fp==NULL && g_warningOutPath!=NULL){
		if((warningOut_fp=fopen(g_warningOutPath, "w"))==NULL){
#ifdef DEBUG_LOG
			//my_printf("Fail to open: %s \n", g_warningOutPath);
			g_warningOutPath = NULL;
			warningOut_fp = NULL;
			return;
#endif
		}
	}
    va_start(vlArgs, strOutputString);
    _vsnprintf_s(strBuffer, sizeof(strBuffer) - 1, 4096, _T(strOutputString), vlArgs);
	fprintf(warningOut_fp, "%s", strBuffer);
	fflush(warningOut_fp);
}
#endif






