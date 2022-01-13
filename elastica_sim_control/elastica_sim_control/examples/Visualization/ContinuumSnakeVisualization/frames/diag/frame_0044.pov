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
    ,<-0.177112369179901,0.5149830703837669,0.0>,0.05
    ,<-0.17804063677420945,0.4950107059184989,0.0>,0.05
    ,<-0.17860442590140513,0.47503664535881107,0.0>,0.05
    ,<-0.17822104398515354,0.45506950067101043,0.0>,0.05
    ,<-0.1761895104445377,0.4352121687792198,0.0>,0.05
    ,<-0.17180442216462444,0.41574668943724885,0.0>,0.05
    ,<-0.16448365777350782,0.3971896890181764,0.0>,0.05
    ,<-0.15390227114873442,0.3802784431588527,0.0>,0.05
    ,<-0.1400985414927469,0.3658689350196051,0.0>,0.05
    ,<-0.12350525329609521,0.35476494335172293,0.0>,0.05
    ,<-0.10487635892522029,0.3475348153568058,0.0>,0.05
    ,<-0.08512233232576182,0.34438585843322717,0.0>,0.05
    ,<-0.06511095775097053,0.3451424316071075,0.0>,0.05
    ,<-0.04550516713770297,0.34932591972862787,0.0>,0.05
    ,<-0.026687646183278534,0.3562936741221587,0.0>,0.05
    ,<-0.008781147783257195,0.36538358314224473,0.0>,0.05
    ,<0.008260911113009358,0.3760286177753493,0.0>,0.05
    ,<0.02454335938216633,0.38781833110922886,0.0>,0.05
    ,<0.040137938470880846,0.4005158264409533,0.0>,0.05
    ,<0.05502345413708489,0.41404843334371066,0.0>,0.05
    ,<0.06905337467759137,0.4284778697701923,0.0>,0.05
    ,<0.08194407045478633,0.4439471034032881,0.0>,0.05
    ,<0.09328454951798044,0.4606015908794982,0.0>,0.05
    ,<0.1025801672211512,0.4784940837051594,0.0>,0.05
    ,<0.10932752445876343,0.4975095813005062,0.0>,0.05
    ,<0.11310367896813363,0.5173423239524837,0.0>,0.05
    ,<0.11364905508769677,0.5375320596232578,0.0>,0.05
    ,<0.11091554716585225,0.5575466017430074,0.0>,0.05
    ,<0.10506011627286845,0.5768780239049976,0.0>,0.05
    ,<0.09638556017101091,0.5951130003461276,0.0>,0.05
    ,<0.08525058588144123,0.6119482858016638,0.0>,0.05
    ,<0.07199220267893804,0.6271534358678816,0.0>,0.05
    ,<0.056891310815309744,0.6405116431152302,0.0>,0.05
    ,<0.04017919103754921,0.6517636850065035,0.0>,0.05
    ,<0.02207756690174089,0.6605711188500658,0.0>,0.05
    ,<0.0028633283400165393,0.6665109414046947,0.0>,0.05
    ,<-0.017056677506943556,0.6691134546389416,0.0>,0.05
    ,<-0.03708755576067239,0.6679515292532354,0.0>,0.05
    ,<-0.05644280496937662,0.6627644275248045,0.0>,0.05
    ,<-0.07421694250204287,0.6535683937550569,0.0>,0.05
    ,<-0.08952203188177887,0.6407118979460856,0.0>,0.05
    ,<-0.10164794444914937,0.624846384075429,0.0>,0.05
    ,<-0.11019230269891725,0.6068135603274799,0.0>,0.05
    ,<-0.11511699755149091,0.5874845964060689,0.0>,0.05
    ,<-0.11671956946632696,0.5676052665587425,0.0>,0.05
    ,<-0.11554135849696592,0.5476942089588921,0.0>,0.05
    ,<-0.11225150647169325,0.5280156654107068,0.0>,0.05
    ,<-0.10754029075918133,0.5086194863292115,0.0>,0.05
    ,<-0.1020348301505852,0.48942281446925845,0.0>,0.05
    ,<-0.09622931543532053,0.4703027076253399,0.0>,0.05
    ,<-0.09041196171179189,0.4511737160562924,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
