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
    ,<-0.14141181162229768,0.5741857894288418,0.0>,0.05
    ,<-0.14911819932597156,0.5557344342862096,0.0>,0.05
    ,<-0.15648740111823994,0.53715428460831,0.0>,0.05
    ,<-0.16296300423839202,0.5182521543446137,0.0>,0.05
    ,<-0.16784131033164176,0.49888359289429346,0.0>,0.05
    ,<-0.17034919398403808,0.4790746414725856,0.0>,0.05
    ,<-0.16974644663835176,0.45912178024640293,0.0>,0.05
    ,<-0.16546023969913418,0.4396285360624952,0.0>,0.05
    ,<-0.15722608735535035,0.42144743385520267,0.0>,0.05
    ,<-0.14518273011793442,0.40552728605441307,0.0>,0.05
    ,<-0.12987019776681555,0.39270896855559856,0.0>,0.05
    ,<-0.11211698043355676,0.3835425995101813,0.0>,0.05
    ,<-0.0928537179913516,0.3781926610324144,0.0>,0.05
    ,<-0.07292386431787375,0.376456539549974,0.0>,0.05
    ,<-0.05295655341026186,0.37787312554212066,0.0>,0.05
    ,<-0.03333175296316825,0.381871675465596,0.0>,0.05
    ,<-0.01422813370592326,0.38791468609431656,0.0>,0.05
    ,<0.004286335559874492,0.39559342034845585,0.0>,0.05
    ,<0.022163973621135192,0.40467015082374636,0.0>,0.05
    ,<0.03930639139001699,0.4150809961617181,0.0>,0.05
    ,<0.05551414310864524,0.4269066576960602,0.0>,0.05
    ,<0.0704543445922122,0.4403111115523162,0.0>,0.05
    ,<0.0836521376670363,0.45544794927102533,0.0>,0.05
    ,<0.09452522018614636,0.472345808855044,0.0>,0.05
    ,<0.10246562507928357,0.4908162301536385,0.0>,0.05
    ,<0.10695064736155703,0.5104239840012181,0.0>,0.05
    ,<0.10765273290694218,0.5305320456025646,0.0>,0.05
    ,<0.10450788953244577,0.5504073865377703,0.0>,0.05
    ,<0.09771454827188118,0.5693469254566221,0.0>,0.05
    ,<0.0876646217268008,0.5867727594492802,0.0>,0.05
    ,<0.07483741850863983,0.6022592738528845,0.0>,0.05
    ,<0.05970999232006786,0.6154945354583006,0.0>,0.05
    ,<0.04271937141145091,0.6262147356776097,0.0>,0.05
    ,<0.024272865035868597,0.6341443548558189,0.0>,0.05
    ,<0.004793843374980414,0.6389632455076784,0.0>,0.05
    ,<-0.015213122889216628,0.6403145037572276,0.0>,0.05
    ,<-0.035100099266118207,0.6378618681787591,0.0>,0.05
    ,<-0.054049109777837445,0.6313954788197511,0.0>,0.05
    ,<-0.07111498251483914,0.6209546694482615,0.0>,0.05
    ,<-0.08534266106865672,0.6069100472573448,0.0>,0.05
    ,<-0.0959284565398764,0.5899649457031033,0.0>,0.05
    ,<-0.10237151545482923,0.5710618566486796,0.0>,0.05
    ,<-0.10456047224451893,0.5512166252188705,0.0>,0.05
    ,<-0.10276593641095763,0.5313338893258145,0.0>,0.05
    ,<-0.0975492947138703,0.5120631541906959,0.0>,0.05
    ,<-0.08962948443349324,0.4937337450423704,0.0>,0.05
    ,<-0.0797558141399868,0.4763735180332745,0.0>,0.05
    ,<-0.06861728837141737,0.45978985742958367,0.0>,0.05
    ,<-0.056791026128170316,0.44368178236428574,0.0>,0.05
    ,<-0.04470990433826044,0.42775568840685824,0.0>,0.05
    ,<-0.032622037097551654,0.4118262246771854,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }