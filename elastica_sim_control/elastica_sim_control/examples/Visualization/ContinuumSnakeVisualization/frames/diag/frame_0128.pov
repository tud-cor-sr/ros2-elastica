#include "../default.inc"

camera{
    location <15.0,10.5,-15.0>
    angle 30
    look_at <4.0,2.7,2.0>
    sky <0,1,0>
    right x*image_width/image_height
}
light_source{
    <15.0,10.5,-15.0>
    color rgb<0.09,0.09,0.1>
}
light_source{
    <1500,2500,-1000>
    color White
}

sphere_sweep {
    linear_spline 51
    ,<0.12955579695823355,0.32525159901764394,0.0>,0.05
    ,<0.11143301348095759,0.31679263916443434,0.0>,0.05
    ,<0.093334938260715,0.30828127524131654,0.0>,0.05
    ,<0.07522663678881922,0.2997934412076737,0.0>,0.05
    ,<0.05705638633158135,0.2914449296410879,0.0>,0.05
    ,<0.03876094179092317,0.2833881163995327,0.0>,0.05
    ,<0.020273122883794584,0.275809233406046,0.0>,0.05
    ,<0.0015318772453664538,0.26892574070863223,0.0>,0.05
    ,<-0.01750534573558775,0.26298297941883,0.0>,0.05
    ,<-0.03684846545631594,0.2582489775346318,0.0>,0.05
    ,<-0.05645996043285331,0.25500613779880066,0.0>,0.05
    ,<-0.07624106130079623,0.2535386952325943,0.0>,0.05
    ,<-0.09602122172987027,0.25411540770960417,0.0>,0.05
    ,<-0.11555348999870556,0.2569679626944169,0.0>,0.05
    ,<-0.13451823336215943,0.2622669248855678,0.0>,0.05
    ,<-0.15253679006864942,0.270098386343986,0.0>,0.05
    ,<-0.1691950094226713,0.28044536450753504,0.0>,0.05
    ,<-0.18407463340299238,0.2931780043184001,0.0>,0.05
    ,<-0.19678853853437395,0.30805556453755756,0.0>,0.05
    ,<-0.20701466227848395,0.3247411890809522,0.0>,0.05
    ,<-0.21452333282977598,0.3428280132509198,0.0>,0.05
    ,<-0.21919385779232284,0.36187293723231384,0.0>,0.05
    ,<-0.2210183014234177,0.38143296707171065,0.0>,0.05
    ,<-0.22009284237933466,0.40109876869090716,0.0>,0.05
    ,<-0.21659932821322112,0.4205210408781136,0.0>,0.05
    ,<-0.21078103550059657,0.4394270727998387,0.0>,0.05
    ,<-0.2029169805744395,0.4576270015751512,0.0>,0.05
    ,<-0.1932984748348699,0.47501102770098713,0.0>,0.05
    ,<-0.18221033146909346,0.4915400521050773,0.0>,0.05
    ,<-0.16991773505197122,0.5072324728611385,0.0>,0.05
    ,<-0.15665856700761277,0.5221496648655831,0.0>,0.05
    ,<-0.14264025660255458,0.536381972243917,0.0>,0.05
    ,<-0.12803987791239244,0.5500363707262762,0.0>,0.05
    ,<-0.11300625524532532,0.5632263118292424,0.0>,0.05
    ,<-0.0976630387279513,0.5760638404997905,0.0>,0.05
    ,<-0.08211200049905419,0.5886538214151691,0.0>,0.05
    ,<-0.06643609809953928,0.6010899807338489,0.0>,0.05
    ,<-0.05070205724165976,0.6134524684859805,0.0>,0.05
    ,<-0.03496244521960423,0.6258066291609904,0.0>,0.05
    ,<-0.019257259761511526,0.6382027536054876,0.0>,0.05
    ,<-0.0036151978045640253,0.6506765636143789,0.0>,0.05
    ,<0.011945297746459124,0.6632502848060234,0.0>,0.05
    ,<0.027415016409181096,0.6759341100251793,0.0>,0.05
    ,<0.04279292764396204,0.6887279729022754,0.0>,0.05
    ,<0.05808494871461939,0.7016234733586121,0.0>,0.05
    ,<0.0733025614711859,0.714605934198302,0.0>,0.05
    ,<0.08846119572062097,0.7276564715008077,0.0>,0.05
    ,<0.1035784837107359,0.7407541154523332,0.0>,0.05
    ,<0.11867234844321153,0.7538778998518676,0.0>,0.05
    ,<0.1337590688433659,0.7670089966680875,0.0>,0.05
    ,<0.14885128884803134,0.7801328266416742,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
