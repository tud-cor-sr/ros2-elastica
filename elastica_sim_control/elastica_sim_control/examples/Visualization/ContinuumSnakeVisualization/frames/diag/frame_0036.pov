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
    ,<-0.196526931219795,0.462702537883758,0.0>,0.05
    ,<-0.1924925233590166,0.44312117758480074,0.0>,0.05
    ,<-0.18810102234467654,0.42363148584816057,0.0>,0.05
    ,<-0.18279588221305992,0.40438401196476204,0.0>,0.05
    ,<-0.17593214740766208,0.3856471910186304,0.0>,0.05
    ,<-0.1669059640469657,0.36785893758660543,0.0>,0.05
    ,<-0.1552884968717559,0.3516463706276273,0.0>,0.05
    ,<-0.1409470062269809,0.3377785913830131,0.0>,0.05
    ,<-0.12411692584505281,0.3270444365122238,0.0>,0.05
    ,<-0.10538674664130541,0.3200852888217874,0.0>,0.05
    ,<-0.08558304433021012,0.31724306615053,0.0>,0.05
    ,<-0.0655860788168849,0.3184843989927992,0.0>,0.05
    ,<-0.04614013760879992,0.3234288783013359,0.0>,0.05
    ,<-0.027724137979958013,0.3314615921185961,0.0>,0.05
    ,<-0.01051728755425915,0.3418784530416011,0.0>,0.05
    ,<0.005545687385083345,0.35401439054532563,0.0>,0.05
    ,<0.020662573755148267,0.3673299641865168,0.0>,0.05
    ,<0.035046745985300665,0.381447254222964,0.0>,0.05
    ,<0.048840695578286894,0.3961516612646214,0.0>,0.05
    ,<0.06206616321045306,0.41137831068118924,0.0>,0.05
    ,<0.07460401890251478,0.4271860047586316,0.0>,0.05
    ,<0.08619523388400466,0.4437128176063446,0.0>,0.05
    ,<0.09646120422771746,0.46110879839849434,0.0>,0.05
    ,<0.10495221382894646,0.4794529439355293,0.0>,0.05
    ,<0.11121779581597131,0.4986869533588869,0.0>,0.05
    ,<0.11488355751926362,0.5185938338452739,0.0>,0.05
    ,<0.11571968618812804,0.5388269745818611,0.0>,0.05
    ,<0.11367920164137926,0.5589781521145956,0.0>,0.05
    ,<0.10889028115093027,0.5786568444410315,0.0>,0.05
    ,<0.10160342499298063,0.5975478058039947,0.0>,0.05
    ,<0.09211008157011893,0.6154224312056528,0.0>,0.05
    ,<0.08066965988558959,0.6321051693932614,0.0>,0.05
    ,<0.06747368712196153,0.6474204877718198,0.0>,0.05
    ,<0.05264597840126665,0.6611405991398669,0.0>,0.05
    ,<0.036274643282503446,0.6729464770469951,0.0>,0.05
    ,<0.018471558371775106,0.6824122557850282,0.0>,0.05
    ,<-0.000549492034511895,0.6890252544814299,0.0>,0.05
    ,<-0.020396738642486846,0.692255015205878,0.0>,0.05
    ,<-0.04046366931741044,0.691665538172475,0.0>,0.05
    ,<-0.0599627583535075,0.6870339104459248,0.0>,0.05
    ,<-0.07803060874595766,0.6784365051377287,0.0>,0.05
    ,<-0.0938776452330958,0.6662669715718913,0.0>,0.05
    ,<-0.10693475812066716,0.6511721322956893,0.0>,0.05
    ,<-0.11694898683565359,0.6339247575705426,0.0>,0.05
    ,<-0.12400237205716325,0.6152774655575575,0.0>,0.05
    ,<-0.12845948441755325,0.5958457548517851,0.0>,0.05
    ,<-0.1308717063844421,0.5760506385395138,0.0>,0.05
    ,<-0.13186979051142833,0.5561245821564,0.0>,0.05
    ,<-0.1320635248298273,0.536162023097652,0.0>,0.05
    ,<-0.13195017951336493,0.5161846436451255,0.0>,0.05
    ,<-0.13182301004920494,0.49619250399477605,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
