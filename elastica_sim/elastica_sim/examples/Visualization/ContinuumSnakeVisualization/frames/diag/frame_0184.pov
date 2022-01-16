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
    ,<0.04272008397947556,-0.007161345929326244,0.0>,0.05
    ,<0.03400313380412444,0.010838603275928761,0.0>,0.05
    ,<0.025279134750857926,0.028834218722722172,0.0>,0.05
    ,<0.01655864720765902,0.046830604623966446,0.0>,0.05
    ,<0.00785780281769121,0.06483552679196578,0.0>,0.05
    ,<-0.0008022539985867953,0.08285906615398062,0.0>,0.05
    ,<-0.009395905684316504,0.1009132266420543,0.0>,0.05
    ,<-0.017893581737280208,0.11901149495329962,0.0>,0.05
    ,<-0.026262261597705838,0.13716835458258414,0.0>,0.05
    ,<-0.034465964828333674,0.15539876074382752,0.0>,0.05
    ,<-0.042466235146555495,0.17371758637343593,0.0>,0.05
    ,<-0.05022262617311526,0.19213905240122436,0.0>,0.05
    ,<-0.05769319725384937,0.2106761579103521,0.0>,0.05
    ,<-0.06483502728059032,0.22934012768600295,0.0>,0.05
    ,<-0.07160475303452658,0.2481398959302978,0.0>,0.05
    ,<-0.07795913622303212,0.26708164551731967,0.0>,0.05
    ,<-0.0838556601843438,0.28616842196787484,0.0>,0.05
    ,<-0.08925315338374595,0.30539984021890965,0.0>,0.05
    ,<-0.09411243259195226,0.32477190014400314,0.0>,0.05
    ,<-0.09839695436065082,0.3442769235900745,0.0>,0.05
    ,<-0.10207345947610108,0.36390362144756194,0.0>,0.05
    ,<-0.10511259187443313,0.3836372940790093,0.0>,0.05
    ,<-0.10748947140805755,0.4034601625174336,0.0>,0.05
    ,<-0.10918419915346504,0.4233518215385392,0.0>,0.05
    ,<-0.1101822748247225,0.4432897994258141,0.0>,0.05
    ,<-0.11047490833562167,0.46325020344862516,0.0>,0.05
    ,<-0.11005921150663332,0.48320842522655943,0.0>,0.05
    ,<-0.10893826105279203,0.5031398766728696,0.0>,0.05
    ,<-0.10712102989568617,0.5230207254007689,0.0>,0.05
    ,<-0.10462219001009884,0.5428285985061623,0.0>,0.05
    ,<-0.10146179590605478,0.5625432255042425,0.0>,0.05
    ,<-0.09766486295512618,0.5821469947262844,0.0>,0.05
    ,<-0.09326085867920654,0.6016254023662376,0.0>,0.05
    ,<-0.08828312754726993,0.6209673791871052,0.0>,0.05
    ,<-0.08276827064522575,0.6401654861889524,0.0>,0.05
    ,<-0.0767555008322234,0.6592159768386304,0.0>,0.05
    ,<-0.07028599185517956,0.6781187293537282,0.0>,0.05
    ,<-0.06340223665730892,0.696877057693695,0.0>,0.05
    ,<-0.05614742615338936,0.7154974141213828,0.0>,0.05
    ,<-0.0485648554503878,0.7339889993488772,0.0>,0.05
    ,<-0.040697360250438346,0.7523632983604562,0.0>,0.05
    ,<-0.03258678232229326,0.7706335610744045,0.0>,0.05
    ,<-0.024273459737282888,0.7888142471698323,0.0>,0.05
    ,<-0.015795735224833646,0.8069204537862006,0.0>,0.05
    ,<-0.00718947461445908,0.8249673435112009,0.0>,0.05
    ,<0.0015124130830092696,0.8429695881895698,0.0>,0.05
    ,<0.010280461892275208,0.8609408416513763,0.0>,0.05
    ,<0.019089148002885525,0.8788932514775674,0.0>,0.05
    ,<0.027917406440802213,0.8968370163595669,0.0>,0.05
    ,<0.036749167441771174,0.9147799914191123,0.0>,0.05
    ,<0.0455739119014635,0.9327273389753923,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }