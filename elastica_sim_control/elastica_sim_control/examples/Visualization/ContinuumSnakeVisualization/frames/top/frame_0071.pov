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
    ,<-0.21273163206152265,0.1998557552231296,0.0>,0.05
    ,<-0.22357570828647133,0.21663839975932106,0.0>,0.05
    ,<-0.23415817910487455,0.23354561605119187,0.0>,0.05
    ,<-0.24418640736390476,0.25075069578655773,0.0>,0.05
    ,<-0.2532774683703752,0.26844014244628506,0.0>,0.05
    ,<-0.2609829785146679,0.2867568891448175,0.0>,0.05
    ,<-0.26681740853239166,0.305744822980023,0.0>,0.05
    ,<-0.27029799668011306,0.3253047899914066,0.0>,0.05
    ,<-0.2709982943816941,0.3451733204206586,0.0>,0.05
    ,<-0.26860884221708364,0.3649333114362269,0.0>,0.05
    ,<-0.2629910052416811,0.3840596747423714,0.0>,0.05
    ,<-0.2542074042056666,0.4019932451516518,0.0>,0.05
    ,<-0.24251692925314622,0.41822656592962687,0.0>,0.05
    ,<-0.22833294881333127,0.4323800933073485,0.0>,0.05
    ,<-0.21215550142868886,0.4442499589783116,0.0>,0.05
    ,<-0.1944962804784042,0.45381775660003176,0.0>,0.05
    ,<-0.1758157762296973,0.46122491718078823,0.0>,0.05
    ,<-0.15648651284237416,0.4667282792821083,0.0>,0.05
    ,<-0.13678267433953595,0.4706539092825783,0.0>,0.05
    ,<-0.11688840175708849,0.473358455076332,0.0>,0.05
    ,<-0.09691620295893666,0.4752016486054378,0.0>,0.05
    ,<-0.07692862025573374,0.47652979183891353,0.0>,0.05
    ,<-0.056958843770150744,0.47766833067046466,0.0>,0.05
    ,<-0.03702833723113758,0.47892123824827393,0.0>,0.05
    ,<-0.017161256304827538,0.4805738987382548,0.0>,0.05
    ,<0.0026035423135446225,0.4828970052662509,0.0>,0.05
    ,<0.022201710534833205,0.48615057595426336,0.0>,0.05
    ,<0.04153253133777895,0.4905873125935349,0.0>,0.05
    ,<0.060446588072237725,0.4964543042789539,0.0>,0.05
    ,<0.07873063388493345,0.5039915162257639,0.0>,0.05
    ,<0.09608859827557953,0.5134243926668226,0.0>,0.05
    ,<0.11212078334670791,0.524943177434218,0.0>,0.05
    ,<0.12630837651272386,0.5386620307925114,0.0>,0.05
    ,<0.13801384392056043,0.554558001869368,0.0>,0.05
    ,<0.14651111896271352,0.572396048602603,0.0>,0.05
    ,<0.151059524823983,0.5916561329031603,0.0>,0.05
    ,<0.15102798415944718,0.611490698512494,0.0>,0.05
    ,<0.14605746258602567,0.6307501236960915,0.0>,0.05
    ,<0.13621461313323652,0.6481065505054877,0.0>,0.05
    ,<0.12206634213663149,0.6622657678578682,0.0>,0.05
    ,<0.10462785905196935,0.6722086083719465,0.0>,0.05
    ,<0.08518356029702054,0.6773795545128708,0.0>,0.05
    ,<0.0650351187669623,0.6777557318707667,0.0>,0.05
    ,<0.04526212200799475,0.6737804052537956,0.0>,0.05
    ,<0.02656844477685664,0.6662018907911411,0.0>,0.05
    ,<0.0092434841782148,0.6558881388163634,0.0>,0.05
    ,<-0.00677822760443625,0.6436777326401898,0.0>,0.05
    ,<-0.021799121579876905,0.6302938426376921,0.0>,0.05
    ,<-0.03622158808048797,0.6163117952076013,0.0>,0.05
    ,<-0.05041806650735887,0.6021491081734449,0.0>,0.05
    ,<-0.06462167592855651,0.5880440839469914,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
