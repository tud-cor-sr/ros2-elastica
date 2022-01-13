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
    ,<-0.15382081089047764,0.5570652712127564,0.0>,0.05
    ,<-0.1594040970744123,0.537865258129659,0.0>,0.05
    ,<-0.16463671185883286,0.5185762483958735,0.0>,0.05
    ,<-0.1689457095440597,0.49906915953684694,0.0>,0.05
    ,<-0.1716182710390077,0.47927960727042923,0.0>,0.05
    ,<-0.17189271350867627,0.45931932253431046,0.0>,0.05
    ,<-0.16907097845992916,0.43956287594784826,0.0>,0.05
    ,<-0.16265522389045892,0.4206679605550258,0.0>,0.05
    ,<-0.15247956863812018,0.4035015806159651,0.0>,0.05
    ,<-0.1387852981538528,0.3889785297235141,0.0>,0.05
    ,<-0.12219486834139753,0.37786066367760235,0.0>,0.05
    ,<-0.10357961810549308,0.3705902367006971,0.0>,0.05
    ,<-0.08386583050964785,0.36721815012639925,0.0>,0.05
    ,<-0.06385127925761187,0.3674438010200169,0.0>,0.05
    ,<-0.04409340656768389,0.3707360598612229,0.0>,0.05
    ,<-0.02489254008880424,0.37648334320366694,0.0>,0.05
    ,<-0.006354928094983815,0.3841291180791133,0.0>,0.05
    ,<0.011508451356687753,0.3932570339846434,0.0>,0.05
    ,<0.028690007088302757,0.40362468399290125,0.0>,0.05
    ,<0.04511756333244779,0.4151617495525073,0.0>,0.05
    ,<0.06060975114945977,0.42793970142334925,0.0>,0.05
    ,<0.07485061283594534,0.4421124519190496,0.0>,0.05
    ,<0.08738752900717822,0.45782721625891526,0.0>,0.05
    ,<0.09766929608255003,0.4751164721891117,0.0>,0.05
    ,<0.10512590194314503,0.49381210804756326,0.0>,0.05
    ,<0.10927204282750626,0.5135187926583403,0.0>,0.05
    ,<0.10980737311946079,0.5336566958300528,0.0>,0.05
    ,<0.10667725303602096,0.5535597049994953,0.0>,0.05
    ,<0.10006908463052737,0.5725910678995495,0.0>,0.05
    ,<0.09034628640003584,0.5902295431982165,0.0>,0.05
    ,<0.07794785822067382,0.6060917189069467,0.0>,0.05
    ,<0.0633034743346543,0.6198930950522901,0.0>,0.05
    ,<0.04679775737224051,0.6313842438232624,0.0>,0.05
    ,<0.02878027687624216,0.6402921444161656,0.0>,0.05
    ,<0.009610363243609872,0.6462862403699541,0.0>,0.05
    ,<-0.010277067784208484,0.6489826694049021,0.0>,0.05
    ,<-0.030304683752294524,0.6479965364355921,0.0>,0.05
    ,<-0.049715945803254946,0.6430441011059935,0.0>,0.05
    ,<-0.06760402365981646,0.6340678187783377,0.0>,0.05
    ,<-0.08301675146126146,0.6213289104596134,0.0>,0.05
    ,<-0.09511255484370394,0.6054261542130227,0.0>,0.05
    ,<-0.10331714564025511,0.5872214808210341,0.0>,0.05
    ,<-0.10742537059476447,0.5676885315005786,0.0>,0.05
    ,<-0.1076139645864453,0.547732741876082,0.0>,0.05
    ,<-0.10436876549141863,0.5280416432341113,0.0>,0.05
    ,<-0.09836246628408378,0.5090071449432022,0.0>,0.05
    ,<-0.09032897808637434,0.4907300228085436,0.0>,0.05
    ,<-0.08096638649025024,0.4730892984477323,0.0>,0.05
    ,<-0.07087447339165108,0.4558465728830089,0.0>,0.05
    ,<-0.06051040317038787,0.43875637655543026,0.0>,0.05
    ,<-0.050137737837823264,0.42166143455544425,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
