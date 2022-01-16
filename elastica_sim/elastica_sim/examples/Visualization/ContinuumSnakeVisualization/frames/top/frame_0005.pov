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
    ,<-0.13724625935236312,0.09281735538497232,0.0>,0.05
    ,<-0.11766690808501737,0.09684567737790122,0.0>,0.05
    ,<-0.09819252101735158,0.10126930969869188,0.0>,0.05
    ,<-0.07895782808182615,0.1065896344579103,0.0>,0.05
    ,<-0.060192548599672345,0.11337290543286059,0.0>,0.05
    ,<-0.0422549372186326,0.12213717568579807,0.0>,0.05
    ,<-0.025634656129766897,0.13324649334966784,0.0>,0.05
    ,<-0.01090451420629427,0.1468260139246591,0.0>,0.05
    ,<0.0013804166385668938,0.16272367011965433,0.0>,0.05
    ,<0.01080775201383503,0.18053861425705453,0.0>,0.05
    ,<0.01721412625930172,0.19971770980870734,0.0>,0.05
    ,<0.020733382590351044,0.21968985191367763,0.0>,0.05
    ,<0.021774347480340638,0.23999110241121246,0.0>,0.05
    ,<0.02094564816222444,0.26033680443897894,0.0>,0.05
    ,<0.01896003858482765,0.28062367650225434,0.0>,0.05
    ,<0.016541833080961806,0.30087280943455497,0.0>,0.05
    ,<0.01434235592289259,0.32114636256378737,0.0>,0.05
    ,<0.01287530391252585,0.3414783550001295,0.0>,0.05
    ,<0.012473891319477438,0.3618473864295298,0.0>,0.05
    ,<0.013268767319708356,0.3821918176513044,0.0>,0.05
    ,<0.015192139051889162,0.4024496391202769,0.0>,0.05
    ,<0.01801064962825454,0.4225957171265837,0.0>,0.05
    ,<0.021383330575736974,0.442655543633619,0.0>,0.05
    ,<0.02493814214619141,0.4626882972994605,0.0>,0.05
    ,<0.028342181157454523,0.48275526372168387,0.0>,0.05
    ,<0.031349473447516256,0.5028947474964073,0.0>,0.05
    ,<0.03382940080907133,0.5231144453682555,0.0>,0.05
    ,<0.03577512048553002,0.5433990797758733,0.0>,0.05
    ,<0.03728975005979708,0.563725167640227,0.0>,0.05
    ,<0.0385487950937307,0.5840723304936878,0.0>,0.05
    ,<0.0397394236860954,0.6044273288121536,0.0>,0.05
    ,<0.04099570983136455,0.6247827052718044,0.0>,0.05
    ,<0.042352420553428326,0.6451364206556082,0.0>,0.05
    ,<0.0437180006354844,0.6654936508713689,0.0>,0.05
    ,<0.04486619746521897,0.6858659210878602,0.0>,0.05
    ,<0.04544622193085653,0.7062589243357155,0.0>,0.05
    ,<0.04501311674799926,0.7266444619439176,0.0>,0.05
    ,<0.043082663970673854,0.7469213683940786,0.0>,0.05
    ,<0.03920470707028536,0.7668830267499754,0.0>,0.05
    ,<0.0330352158136222,0.7862110958762033,0.0>,0.05
    ,<0.024396113610885388,0.8045064794219704,0.0>,0.05
    ,<0.013307633011976622,0.8213553890952876,0.0>,0.05
    ,<-1.9199096397052684e-05,0.8364125110479832,0.0>,0.05
    ,<-0.015230600453301261,0.8494763686884238,0.0>,0.05
    ,<-0.0319031683319368,0.8605329236590669,0.0>,0.05
    ,<-0.0496192854313016,0.8697583129446065,0.0>,0.05
    ,<-0.06802142912052793,0.8774827623567097,0.0>,0.05
    ,<-0.08683685984813004,0.8841290324772221,0.0>,0.05
    ,<-0.10587383043093435,0.8901356379582676,0.0>,0.05
    ,<-0.1250080258287776,0.8958732415354635,0.0>,0.05
    ,<-0.14417530145926252,0.9015549732981908,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }