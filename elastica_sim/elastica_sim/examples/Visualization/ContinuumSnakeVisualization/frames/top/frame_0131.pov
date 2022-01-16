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
    ,<0.14714781882828437,0.2760106366869227,0.0>,0.05
    ,<0.1275266095724739,0.2721410224220072,0.0>,0.05
    ,<0.1079159339522405,0.26822638629219653,0.0>,0.05
    ,<0.08830318666828811,0.26433217054486574,0.0>,0.05
    ,<0.06867011826767778,0.2605578021431243,0.0>,0.05
    ,<0.04899575478309818,0.25703307682678356,0.0>,0.05
    ,<0.029260801179608643,0.2539145693152487,0.0>,0.05
    ,<0.009453613710481003,0.251381844973953,0.0>,0.05
    ,<-0.010422504272227646,0.2496330808220571,0.0>,0.05
    ,<-0.03034106248039527,0.2488796004457265,0.0>,0.05
    ,<-0.05024394320155251,0.24933877788376368,0.0>,0.05
    ,<-0.07003367915710938,0.2512248680784334,0.0>,0.05
    ,<-0.08956769411575952,0.25473758869778634,0.0>,0.05
    ,<-0.10865579222024464,0.2600487259316526,0.0>,0.05
    ,<-0.12706216265574,0.2672876313501365,0.0>,0.05
    ,<-0.14451273481580257,0.27652707247159364,0.0>,0.05
    ,<-0.16070807326093522,0.2877713754912793,0.0>,0.05
    ,<-0.1753410713723576,0.30094892518464766,0.0>,0.05
    ,<-0.18811774679247223,0.3159107851679412,0.0>,0.05
    ,<-0.19877869272022894,0.3324364404834548,0.0>,0.05
    ,<-0.20711836539245632,0.3502465562696394,0.0>,0.05
    ,<-0.21299962495135139,0.36902150014167273,0.0>,0.05
    ,<-0.21636164477447586,0.3884233510838982,0.0>,0.05
    ,<-0.21722043962654486,0.4081186309753735,0.0>,0.05
    ,<-0.21566245236299758,0.42779895023497827,0.0>,0.05
    ,<-0.21183268660831645,0.44719735957142587,0.0>,0.05
    ,<-0.20591951523792407,0.4660990563257824,0.0>,0.05
    ,<-0.19813844508322118,0.4843461412159663,0.0>,0.05
    ,<-0.1887168391546526,0.5018370072684057,0.0>,0.05
    ,<-0.17788101804411724,0.518521528387415,0.0>,0.05
    ,<-0.16584647408662173,0.5343935122874209,0.0>,0.05
    ,<-0.15281133967719734,0.5494817746633619,0.0>,0.05
    ,<-0.13895276052929187,0.5638410246785515,0.0>,0.05
    ,<-0.12442562968106226,0.5775433254182739,0.0>,0.05
    ,<-0.10936297946353646,0.5906706676901722,0.0>,0.05
    ,<-0.09387746387354606,0.6033088158134342,0.0>,0.05
    ,<-0.07806336447843826,0.6155425071009514,0.0>,0.05
    ,<-0.06199880963186503,0.6274518537069114,0.0>,0.05
    ,<-0.04574789305019512,0.6391098551826017,0.0>,0.05
    ,<-0.029362629721892502,0.6505807936109854,0.0>,0.05
    ,<-0.012884618985964616,0.6619194064908079,0.0>,0.05
    ,<0.003653501422417044,0.6731706300984809,0.0>,0.05
    ,<0.020226856000701374,0.6843698448618187,0.0>,0.05
    ,<0.03681724867171923,0.6955434544848464,0.0>,0.05
    ,<0.053412228288301854,0.7067097686890658,0.0>,0.05
    ,<0.0700041647125347,0.7178800540754162,0.0>,0.05
    ,<0.08658938832855932,0.7290597539762362,0.0>,0.05
    ,<0.1031672774991949,0.7402497671878709,0.0>,0.05
    ,<0.1197393756603671,0.7514478114220466,0.0>,0.05
    ,<0.1363084443727198,0.7626497811915456,0.0>,0.05
    ,<0.1528775537887339,0.77385114500272,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }