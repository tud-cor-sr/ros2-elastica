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
    ,<-0.07659949277071823,0.4750713261962267,0.0>,0.05
    ,<-0.06868223963621244,0.4567070468644425,0.0>,0.05
    ,<-0.06099340887572022,0.43824968639259837,0.0>,0.05
    ,<-0.053920712956497335,0.4195509588113575,0.0>,0.05
    ,<-0.0479792270159706,0.40046556781239334,0.0>,0.05
    ,<-0.04377664956229931,0.38092604421468446,0.0>,0.05
    ,<-0.041969636785027686,0.36102386469356035,0.0>,0.05
    ,<-0.0431970232939468,0.34107997725795874,0.0>,0.05
    ,<-0.04798413920608974,0.3216831477761479,0.0>,0.05
    ,<-0.05662764716780317,0.3036745177762898,0.0>,0.05
    ,<-0.06908701345110538,0.28806573894695175,0.0>,0.05
    ,<-0.08491842297477474,0.2758969695434921,0.0>,0.05
    ,<-0.10328192343568901,0.2680641411843554,0.0>,0.05
    ,<-0.12303151525343572,0.2651610683782416,0.0>,0.05
    ,<-0.14286884750019638,0.2673810352299745,0.0>,0.05
    ,<-0.16151808404794935,0.27450273586631924,0.0>,0.05
    ,<-0.17787407333612326,0.2859552031915296,0.0>,0.05
    ,<-0.1910941892818455,0.30092747437364475,0.0>,0.05
    ,<-0.2006312351275013,0.31848694743905703,0.0>,0.05
    ,<-0.20621999710603212,0.3376826625268851,0.0>,0.05
    ,<-0.20783544576040505,0.3576217740342039,0.0>,0.05
    ,<-0.20563939063388859,0.3775177692252074,0.0>,0.05
    ,<-0.19992766427550024,0.3967154738104692,0.0>,0.05
    ,<-0.19108428566594518,0.4147004979707024,0.0>,0.05
    ,<-0.17954326233017343,0.4310997069310881,0.0>,0.05
    ,<-0.16575684797194523,0.44567658621297473,0.0>,0.05
    ,<-0.15016960059955953,0.45832402236872477,0.0>,0.05
    ,<-0.13319756301109847,0.46905619507283036,0.0>,0.05
    ,<-0.11521227652495122,0.47800071507618047,0.0>,0.05
    ,<-0.09653007931398976,0.48539188180534854,0.0>,0.05
    ,<-0.07740812947802554,0.49156574704506434,0.0>,0.05
    ,<-0.05804883854552483,0.49695351446820557,0.0>,0.05
    ,<-0.038616375509636536,0.5020688224515135,0.0>,0.05
    ,<-0.01926884330046224,0.5074878581133454,0.0>,0.05
    ,<-0.0002048659074052263,0.5138192249756018,0.0>,0.05
    ,<0.01828351581950894,0.5216577748140165,0.0>,0.05
    ,<0.03576806944999819,0.5315167874910975,0.0>,0.05
    ,<0.051675673292270935,0.5437400935081282,0.0>,0.05
    ,<0.0653336327864167,0.5584163354272097,0.0>,0.05
    ,<0.07606796480621106,0.5753316859190729,0.0>,0.05
    ,<0.0833313448031469,0.5939869885732955,0.0>,0.05
    ,<0.08681933799110672,0.613686820821819,0.0>,0.05
    ,<0.0865335235480812,0.6336801863027933,0.0>,0.05
    ,<0.0827709290921174,0.6533104562588681,0.0>,0.05
    ,<0.07604975871332482,0.6721292492766282,0.0>,0.05
    ,<0.06700428496405032,0.6899459531295663,0.0>,0.05
    ,<0.056285531520310024,0.7068099811505325,0.0>,0.05
    ,<0.044490179771067555,0.7229425809927913,0.0>,0.05
    ,<0.0321185024055156,0.738642409764172,0.0>,0.05
    ,<0.019544840876301147,0.7541866654793637,0.0>,0.05
    ,<0.006978668166119112,0.7697429607660503,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
