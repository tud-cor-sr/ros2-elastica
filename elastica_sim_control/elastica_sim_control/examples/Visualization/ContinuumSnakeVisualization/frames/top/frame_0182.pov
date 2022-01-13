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
    ,<0.0473960769218555,-0.003918524020220128,0.0>,0.05
    ,<0.03824491361310911,0.013864563428985168,0.0>,0.05
    ,<0.029086472221344938,0.031642893455292496,0.0>,0.05
    ,<0.019931698691746214,0.04942208775252517,0.0>,0.05
    ,<0.010797324741449288,0.06721070472531847,0.0>,0.05
    ,<0.001705296175197291,0.08501986041323406,0.0>,0.05
    ,<-0.007317776948956425,0.10286279938791995,0.0>,0.05
    ,<-0.016241147712123614,0.1207544126612221,0.0>,0.05
    ,<-0.02503043992817055,0.13871070494200957,0.0>,0.05
    ,<-0.033648141466040456,0.1567482181474687,0.0>,0.05
    ,<-0.042054091164056916,0.17488342198251605,0.0>,0.05
    ,<-0.050205968782816544,0.1931320857151324,0.0>,0.05
    ,<-0.058059798093766116,0.21150864803540942,0.0>,0.05
    ,<-0.06557047278413496,0.23002560409010359,0.0>,0.05
    ,<-0.07269231330738848,0.24869293037893545,0.0>,0.05
    ,<-0.07937966013648624,0.267517569079919,0.0>,0.05
    ,<-0.08558750519449548,0.2865029933949221,0.0>,0.05
    ,<-0.09127215875441654,0.3056488745129184,0.0>,0.05
    ,<-0.09639194412711119,0.32495086863012707,0.0>,0.05
    ,<-0.10090790738848317,0.3444005390486052,0.0>,0.05
    ,<-0.10478452468900627,0.36398542368988696,0.0>,0.05
    ,<-0.10799038581549954,0.3836892525201306,0.0>,0.05
    ,<-0.11049883008227675,0.40349231263387364,0.0>,0.05
    ,<-0.11228850967902737,0.42337195146728623,0.0>,0.05
    ,<-0.11334385652258329,0.44330320129964085,0.0>,0.05
    ,<-0.11365543149984061,0.46325950141414063,0.0>,0.05
    ,<-0.11322013960213458,0.483213488589777,0.0>,0.05
    ,<-0.11204130049620413,0.5031378224921982,0.0>,0.05
    ,<-0.11012857104873125,0.5230060103966224,0.0>,0.05
    ,<-0.10749772360738406,0.542793195716069,0.0>,0.05
    ,<-0.10417029079049975,0.5624768770158421,0.0>,0.05
    ,<-0.10017309353954365,0.5820375283641915,0.0>,0.05
    ,<-0.09553767374315211,0.601459097615302,0.0>,0.05
    ,<-0.0902996555136644,0.6207293660367809,0.0>,0.05
    ,<-0.08449806004600523,0.6398401600119803,0.0>,0.05
    ,<-0.07817459797128908,0.6587874128108122,0.0>,0.05
    ,<-0.07137296046581518,0.6775710811408707,0.0>,0.05
    ,<-0.06413812645602573,0.6961949269877364,0.0>,0.05
    ,<-0.05651569851936569,0.7146661798867839,0.0>,0.05
    ,<-0.048551274994831625,0.7329950981326825,0.0>,0.05
    ,<-0.040289860838753265,0.7511944495388476,0.0>,0.05
    ,<-0.03177531528401472,0.7692789333065875,0.0>,0.05
    ,<-0.0230498306915643,0.7872645645015944,0.0>,0.05
    ,<-0.014153434329496865,0.8051680417265915,0.0>,0.05
    ,<-0.005123503284572758,0.8230061169671147,0.0>,0.05
    ,<0.004005717683828707,0.8407949843773798,0.0>,0.05
    ,<0.01320360486542323,0.8585497020171619,0.0>,0.05
    ,<0.0224436588104973,0.8762836572502051,0.0>,0.05
    ,<0.03170402758908899,0.8940080826290545,0.0>,0.05
    ,<0.04096805274984834,0.9117316245529589,0.0>,0.05
    ,<0.05022483737241912,0.9294599617217344,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
