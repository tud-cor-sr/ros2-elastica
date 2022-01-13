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
    ,<-0.21392147715735474,0.3635260479810523,0.0>,0.05
    ,<-0.20237531126094474,0.3472062546467074,0.0>,0.05
    ,<-0.19053260661591673,0.3311212240518633,0.0>,0.05
    ,<-0.177953910728745,0.3156236304520727,0.0>,0.05
    ,<-0.16416932671960718,0.30120207284401546,0.0>,0.05
    ,<-0.148812263409121,0.28847345273682473,0.0>,0.05
    ,<-0.1317415384737637,0.2781460642305479,0.0>,0.05
    ,<-0.11312375723439595,0.2709311802738111,0.0>,0.05
    ,<-0.09344348729311498,0.26741150030044814,0.0>,0.05
    ,<-0.07342262924928665,0.26790697556873405,0.0>,0.05
    ,<-0.053863279172570264,0.2723925237144927,0.0>,0.05
    ,<-0.035463773951726715,0.28050537544949194,0.0>,0.05
    ,<-0.01867185710383165,0.29163929233772873,0.0>,0.05
    ,<-0.0036210591466461334,0.3050838531557093,0.0>,0.05
    ,<0.00984110577278288,0.32015425754449606,0.0>,0.05
    ,<0.022056619983659716,0.33627566118835955,0.0>,0.05
    ,<0.03342836942110438,0.35301834575737534,0.0>,0.05
    ,<0.04430686288166781,0.3700950949107585,0.0>,0.05
    ,<0.05491730630539428,0.38734564732415855,0.0>,0.05
    ,<0.06532584697133484,0.40472372432438386,0.0>,0.05
    ,<0.07543632418400066,0.4222831094216607,0.0>,0.05
    ,<0.08500817403294897,0.4401509573785309,0.0>,0.05
    ,<0.09369116048080557,0.45847975338658387,0.0>,0.05
    ,<0.10108099072056431,0.4773819151270625,0.0>,0.05
    ,<0.10678497423713518,0.4968744064850592,0.0>,0.05
    ,<0.11048416435732575,0.516857770273062,0.0>,0.05
    ,<0.11198465728278635,0.537134493647875,0.0>,0.05
    ,<0.11124423530280565,0.5574576969758533,0.0>,0.05
    ,<0.10836341725391586,0.5775890443135127,0.0>,0.05
    ,<0.10354027368345242,0.5973409759776641,0.0>,0.05
    ,<0.096998599370566,0.6165849007756149,0.0>,0.05
    ,<0.0889187585386298,0.6352257510155574,0.0>,0.05
    ,<0.07939732835438543,0.6531611594730928,0.0>,0.05
    ,<0.06843549566711878,0.6702390134128012,0.0>,0.05
    ,<0.055955192192721465,0.6862202280963791,0.0>,0.05
    ,<0.04184355778520216,0.7007519321936073,0.0>,0.05
    ,<0.026024696798615393,0.7133606932346549,0.0>,0.05
    ,<0.00854891208778692,0.7234825818589444,0.0>,0.05
    ,<-0.010331086162103735,0.7305387915217655,0.0>,0.05
    ,<-0.030135088780880788,0.7340405596801955,0.0>,0.05
    ,<-0.05019787255155541,0.7336975808955165,0.0>,0.05
    ,<-0.06977448390361297,0.7294939267618537,0.0>,0.05
    ,<-0.08817909583728392,0.7217015663701828,0.0>,0.05
    ,<-0.10491363427177934,0.7108249055221002,0.0>,0.05
    ,<-0.11974730034975854,0.6974977664325708,0.0>,0.05
    ,<-0.13272934096743494,0.6823709181088898,0.0>,0.05
    ,<-0.14414172388353544,0.6660255755144193,0.0>,0.05
    ,<-0.15441293068179068,0.648930115065032,0.0>,0.05
    ,<-0.16401490531163898,0.6314345587927922,0.0>,0.05
    ,<-0.17335757265632598,0.61378056996507,0.0>,0.05
    ,<-0.18268727207153915,0.5960999603679604,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
