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
    ,<-0.023713681189012976,0.40391612612626526,0.0>,0.05
    ,<-0.03141900638603377,0.3854667166056697,0.0>,0.05
    ,<-0.039352893848283194,0.3671275020180977,0.0>,0.05
    ,<-0.04787885078500982,0.34906821956806705,0.0>,0.05
    ,<-0.057442243731746216,0.33154714727244194,0.0>,0.05
    ,<-0.06849553863118275,0.31493484591118914,0.0>,0.05
    ,<-0.08141502417833275,0.29973535597552203,0.0>,0.05
    ,<-0.0964119593991842,0.2865849509356917,0.0>,0.05
    ,<-0.11344926397072644,0.27621202677918655,0.0>,0.05
    ,<-0.13218351291339112,0.26935243595709424,0.0>,0.05
    ,<-0.15195510866984124,0.266631026089659,0.0>,0.05
    ,<-0.17184201264067317,0.26843693701388094,0.0>,0.05
    ,<-0.19077432852877463,0.27482918044693755,0.0>,0.05
    ,<-0.20768518421349103,0.2855036812656667,0.0>,0.05
    ,<-0.22165852831144514,0.29983351027594807,0.0>,0.05
    ,<-0.23203483550692333,0.31696868993686966,0.0>,0.05
    ,<-0.2384519795912844,0.3359623131083782,0.0>,0.05
    ,<-0.24082736927905446,0.35588451841887375,0.0>,0.05
    ,<-0.2393040172401715,0.37590303966048055,0.0>,0.05
    ,<-0.234183018290361,0.39532669801132264,0.0>,0.05
    ,<-0.22585935542231828,0.41361808499203595,0.0>,0.05
    ,<-0.214770345017852,0.4303860035120009,0.0>,0.05
    ,<-0.20135939821272486,0.4453682557962148,0.0>,0.05
    ,<-0.1860533245925037,0.45841299677226777,0.0>,0.05
    ,<-0.16924851802534355,0.4694626274778304,0.0>,0.05
    ,<-0.15130211250414044,0.47854116220423054,0.0>,0.05
    ,<-0.13252602276401468,0.4857453245247387,0.0>,0.05
    ,<-0.11318262816735589,0.4912391453359453,0.0>,0.05
    ,<-0.09348177823539797,0.49525171248621064,0.0>,0.05
    ,<-0.07357981929545268,0.49807788614827675,0.0>,0.05
    ,<-0.053582498117308544,0.5000819504175158,0.0>,0.05
    ,<-0.033554713589551054,0.5017001421340166,0.0>,0.05
    ,<-0.013541632726183337,0.503437302986182,0.0>,0.05
    ,<0.00639543281225438,0.5058560146265383,0.0>,0.05
    ,<0.026129525957965243,0.509553892569001,0.0>,0.05
    ,<0.04541277152478946,0.5151217702002512,0.0>,0.05
    ,<0.06383191969673241,0.5230759105490892,0.0>,0.05
    ,<0.08079967359639503,0.5337653848097436,0.0>,0.05
    ,<0.09560390371687223,0.5472781920874285,0.0>,0.05
    ,<0.10751491892194266,0.5633859106652134,0.0>,0.05
    ,<0.11592580810612531,0.5815564626299699,0.0>,0.05
    ,<0.1204811909767251,0.6010446299112863,0.0>,0.05
    ,<0.12114935537695933,0.621039545524563,0.0>,0.05
    ,<0.11821495020965762,0.6408239571225931,0.0>,0.05
    ,<0.11220238461422377,0.659896392798906,0.0>,0.05
    ,<0.10376497085103162,0.6780254609517639,0.0>,0.05
    ,<0.09357915088580794,0.6952328717474476,0.0>,0.05
    ,<0.08226811199154095,0.7117231030634407,0.0>,0.05
    ,<0.07035610781846158,0.7277856744531949,0.0>,0.05
    ,<0.05823651724978221,0.7436934220966845,0.0>,0.05
    ,<0.04613092201480588,0.7596130581569915,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
