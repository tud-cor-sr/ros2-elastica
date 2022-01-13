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
    ,<0.0830636289117411,0.028399824274355442,0.0>,0.05
    ,<0.0705885626264747,0.04403131883496504,0.0>,0.05
    ,<0.05810506177423987,0.05965422743154479,0.0>,0.05
    ,<0.04562627927778241,0.07527902323607506,0.0>,0.05
    ,<0.033172440936960784,0.0909217366616158,0.0>,0.05
    ,<0.020770244120581434,0.10660329161926554,0.0>,0.05
    ,<0.008452315204022003,0.12234875328716671,0.0>,0.05
    ,<-0.0037432729906452895,0.13818647867435435,0.0>,0.05
    ,<-0.015773431288466106,0.15414716920866173,0.0>,0.05
    ,<-0.02759044749470638,0.17026283158741548,0.0>,0.05
    ,<-0.03914236577468889,0.18656565956287235,0.0>,0.05
    ,<-0.05037338040397962,0.2030868555241016,0.0>,0.05
    ,<-0.061224275618887436,0.21985541693838467,0.0>,0.05
    ,<-0.07163294409126127,0.23689691900414786,0.0>,0.05
    ,<-0.08153501432081518,0.25423233110108157,0.0>,0.05
    ,<-0.09086461171558605,0.27187691039501793,0.0>,0.05
    ,<-0.09955526927585694,0.2898392206102562,0.0>,0.05
    ,<-0.10754099187309459,0.3081203266742731,0.0>,0.05
    ,<-0.11475746375156122,0.32671321573243906,0.0>,0.05
    ,<-0.12114337310746875,0.3456024910666456,0.0>,0.05
    ,<-0.12664181181112155,0.3647643771134391,0.0>,0.05
    ,<-0.13120169419890612,0.3841670608971319,0.0>,0.05
    ,<-0.13477912812069343,0.4037713781596252,0.0>,0.05
    ,<-0.1373386656898337,0.42353183233346464,0.0>,0.05
    ,<-0.13885436162958417,0.4433979128907038,0.0>,0.05
    ,<-0.1393105742789585,0.463315658593309,0.0>,0.05
    ,<-0.1387024579293124,0.48322939299268436,0.0>,0.05
    ,<-0.1370361140839144,0.5030835462717493,0.0>,0.05
    ,<-0.1343283915946657,0.5228244707554663,0.0>,0.05
    ,<-0.1306063490691974,0.5424021579839992,0.0>,0.05
    ,<-0.1259064149359303,0.5617717730545738,0.0>,0.05
    ,<-0.12027329879532417,0.5808949360541896,0.0>,0.05
    ,<-0.11375872040676313,0.5997406991298618,0.0>,0.05
    ,<-0.10642002885812178,0.6182861889684196,0.0>,0.05
    ,<-0.09831878399275845,0.6365169059372816,0.0>,0.05
    ,<-0.08951936566608999,0.6544266908679353,0.0>,0.05
    ,<-0.08008766513732957,0.6720173868797811,0.0>,0.05
    ,<-0.07008989849982791,0.6892982358149427,0.0>,0.05
    ,<-0.059591566263233586,0.7062850564680172,0.0>,0.05
    ,<-0.04865656763896612,0.7229992551107259,0.0>,0.05
    ,<-0.03734646408586041,0.7394667184820731,0.0>,0.05
    ,<-0.02571987519898129,0.7557166363322109,0.0>,0.05
    ,<-0.013831981638364892,0.7717802957003145,0.0>,0.05
    ,<-0.0017341046999794715,0.7876898832087937,0.0>,0.05
    ,<0.010526669752333812,0.8034773253978829,0.0>,0.05
    ,<0.022907854238960756,0.8191731908869392,0.0>,0.05
    ,<0.035371937215809494,0.834805672061106,0.0>,0.05
    ,<0.047886797409338704,0.8503996579492575,0.0>,0.05
    ,<0.06042617010843973,0.8659759037361028,0.0>,0.05
    ,<0.07297017258107368,0.8815502955947794,0.0>,0.05
    ,<0.08550588721783825,0.8971332019012715,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
