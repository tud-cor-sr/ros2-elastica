#include "../default.inc"

camera{
    location <0,15,3>
    angle 30
    look_at <0.0,0,3>
    sky <-1,0,0>
    right x*image_width/image_height
}
light_source{
    <0.0,8.0,5.0>
    color rgb<0.09,0.09,0.1>
}
light_source{
    <1500,2500,-1000>
    color White
}

sphere_sweep {
    linear_spline 51
    ,<0.015921617443954317,-0.022127666110017964,0.0>,0.05
    ,<0.009689253981105764,-0.003123756858526637,0.0>,0.05
    ,<0.0034514411938194557,0.015877889179812504,0.0>,0.05
    ,<-0.0027838052716437534,0.03487989590811347,0.0>,0.05
    ,<-0.009004280041624164,0.0538862435442366,0.0>,0.05
    ,<-0.015194036242344144,0.07290208479507876,0.0>,0.05
    ,<-0.021333826721287457,0.09193353710791764,0.0>,0.05
    ,<-0.027401537495558383,0.11098745018612921,0.0>,0.05
    ,<-0.033372613538763994,0.13007115070172598,0.0>,0.05
    ,<-0.03922047777788791,0.1491921684530962,0.0>,0.05
    ,<-0.04491694474679603,0.16835795010897484,0.0>,0.05
    ,<-0.05043263067008262,0.18757556815515117,0.0>,0.05
    ,<-0.05573736178013073,0.20685143372479808,0.0>,0.05
    ,<-0.06080058237199307,0.22619102264750304,0.0>,0.05
    ,<-0.06559176347205097,0.24559862429466228,0.0>,0.05
    ,<-0.07008081206277934,0.26507712262354627,0.0>,0.05
    ,<-0.07423847962265724,0.28462781822541244,0.0>,0.05
    ,<-0.07803676738909619,0.3042502991638576,0.0>,0.05
    ,<-0.08144932433786548,0.32394236696110407,0.0>,0.05
    ,<-0.08445183251397817,0.34370002228039853,0.0>,0.05
    ,<-0.08702237317137025,0.363517512714923,0.0>,0.05
    ,<-0.08914176630102442,0.38338744270436226,0.0>,0.05
    ,<-0.09079387565157526,0.4033009430636408,0.0>,0.05
    ,<-0.09196587134637887,0.42324789504932764,0.0>,0.05
    ,<-0.09264844271271842,0.44321720144920557,0.0>,0.05
    ,<-0.0928359549547725,0.4631970950048506,0.0>,0.05
    ,<-0.09252654476965395,0.48317547270412536,0.0>,0.05
    ,<-0.09172215183056347,0.503140243229084,0.0>,0.05
    ,<-0.0904284851140952,0.5230796741988666,0.0>,0.05
    ,<-0.08865492517902791,0.5429827258547437,0.0>,0.05
    ,<-0.08641436555274662,0.562839358492648,0.0>,0.05
    ,<-0.08372299819814882,0.5826408022123184,0.0>,0.05
    ,<-0.08060004949021393,0.60237977933443,0.0>,0.05
    ,<-0.07706747413350348,0.6220506720192375,0.0>,0.05
    ,<-0.07314961494803625,0.6416496300635486,0.0>,0.05
    ,<-0.06887283643475368,0.6611746164116307,0.0>,0.05
    ,<-0.06426513954019906,0.6806253904491966,0.0>,0.05
    ,<-0.05935576414744408,0.7000034315328624,0.0>,0.05
    ,<-0.05417478462936349,0.7193118073376651,0.0>,0.05
    ,<-0.0487527024307698,0.7385549934055825,0.0>,0.05
    ,<-0.04312003822280123,0.757738651697394,0.0>,0.05
    ,<-0.03730692481577341,0.7768693769602641,0.0>,0.05
    ,<-0.03134270082989075,0.7959544203128363,0.0>,0.05
    ,<-0.025255504188802933,0.815001399618513,0.0>,0.05
    ,<-0.019071863873975036,0.8340180059709918,0.0>,0.05
    ,<-0.012816288083729909,0.8530117149598039,0.0>,0.05
    ,<-0.006510846976547789,0.8719895103193838,0.0>,0.05
    ,<-0.00017474851431980182,0.8909576260917176,0.0>,0.05
    ,<0.006176093492136775,0.9099213115438695,0.0>,0.05
    ,<0.012529499259034623,0.9288846207707508,0.0>,0.05
    ,<0.01887747005553921,0.9478502261743978,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
