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
    ,<0.09908027684329299,0.04842176697941111,0.0>,0.05
    ,<0.08509923562959529,0.06272201558500695,0.0>,0.05
    ,<0.07110962036855739,0.07701156231102607,0.0>,0.05
    ,<0.05712507735090354,0.09130371514389325,0.0>,0.05
    ,<0.043166653100319155,0.10561889358738455,0.0>,0.05
    ,<0.029262225234103354,0.11998381166219477,0.0>,0.05
    ,<0.015446020700082512,0.1344305528175602,0.0>,0.05
    ,<0.0017582240103704901,0.14899552386308057,0.0>,0.05
    ,<-0.01175533475683996,0.16371828363960303,0.0>,0.05
    ,<-0.025043434545721385,0.17864024974370296,0.0>,0.05
    ,<-0.03804969376096198,0.19380329391108186,0.0>,0.05
    ,<-0.05071281870281213,0.2092482441996276,0.0>,0.05
    ,<-0.06296691830831123,0.22501332048016248,0.0>,0.05
    ,<-0.07474193605029421,0.2411325390917412,0.0>,0.05
    ,<-0.08596424806009363,0.25763413278487857,0.0>,0.05
    ,<-0.09655746972141008,0.2745390425590471,0.0>,0.05
    ,<-0.10644350084822117,0.29185954768409156,0.0>,0.05
    ,<-0.11554382215887642,0.30959810748091454,0.0>,0.05
    ,<-0.1237810338709843,0.32774649163433506,0.0>,0.05
    ,<-0.13108060228167875,0.3462852731024191,0.0>,0.05
    ,<-0.13737275439793767,0.365183747745853,0.0>,0.05
    ,<-0.1425944368142868,0.38440032691669784,0.0>,0.05
    ,<-0.14669123621722108,0.40388342383424913,0.0>,0.05
    ,<-0.1496191480712232,0.4235728231991423,0.0>,0.05
    ,<-0.1513460795043978,0.4434014889243501,0.0>,0.05
    ,<-0.15185298329950148,0.46329773074152614,0.0>,0.05
    ,<-0.1511345418424294,0.48318762077927174,0.0>,0.05
    ,<-0.149199350935121,0.5029975297943522,0.0>,0.05
    ,<-0.14606959019417307,0.5226566423637794,0.0>,0.05
    ,<-0.14178020508117242,0.5420993124621319,0.0>,0.05
    ,<-0.13637766095847267,0.5612671351068358,0.0>,0.05
    ,<-0.12991835793070247,0.5801106342584476,0.0>,0.05
    ,<-0.12246681372271542,0.5985904986594793,0.0>,0.05
    ,<-0.11409372904520537,0.6166783318953328,0.0>,0.05
    ,<-0.1048740459894947,0.6343569167026473,0.0>,0.05
    ,<-0.09488509654039996,0.6516200230614615,0.0>,0.05
    ,<-0.08420491784894912,0.6684718125330837,0.0>,0.05
    ,<-0.0729107865025996,0.684925906452191,0.0>,0.05
    ,<-0.06107799865026232,0.701004192985649,0.0>,0.05
    ,<-0.048778899088637646,0.7167354486094054,0.0>,0.05
    ,<-0.03608214217252252,0.7321538448225043,0.0>,0.05
    ,<-0.02305215188933679,0.7472974026162401,0.0>,0.05
    ,<-0.009748738060301397,0.7622064471249413,0.0>,0.05
    ,<0.0037731796250196374,0.7769221043207818,0.0>,0.05
    ,<0.017463788702636433,0.791484871630469,0.0>,0.05
    ,<0.031278587824942757,0.8059332853616183,0.0>,0.05
    ,<0.045178616721501155,0.8203026999877531,0.0>,0.05
    ,<0.05913074271793581,0.8346241872961154,0.0>,0.05
    ,<0.07310802393337594,0.8489235567402572,0.0>,0.05
    ,<0.08709015788336866,0.8632204913767256,0.0>,0.05
    ,<0.1010640123808406,0.8775277860076726,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }