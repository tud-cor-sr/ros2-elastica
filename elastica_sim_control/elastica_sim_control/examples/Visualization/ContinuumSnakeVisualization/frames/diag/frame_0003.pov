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
    ,<-0.10780661100226811,0.06272908975741184,0.0>,0.05
    ,<-0.08943000033929392,0.07059708164417385,0.0>,0.05
    ,<-0.0712383033841655,0.07884161491339106,0.0>,0.05
    ,<-0.053456026596914064,0.08791196467390931,0.0>,0.05
    ,<-0.03640764335100434,0.09829792108687893,0.0>,0.05
    ,<-0.020524453263881328,0.11041808193997511,0.0>,0.05
    ,<-0.006325650346789108,0.12452204741297022,0.0>,0.05
    ,<0.0056438847655327085,0.1406239428133006,0.0>,0.05
    ,<0.014912978984425238,0.15848813052089508,0.0>,0.05
    ,<0.021192495781054393,0.177680574773393,0.0>,0.05
    ,<0.024453104813223943,0.19767578564925536,0.0>,0.05
    ,<0.024944317857517523,0.21798619220931445,0.0>,0.05
    ,<0.02315251371179359,0.23826750830632823,0.0>,0.05
    ,<0.019721420991778846,0.258366737574192,0.0>,0.05
    ,<0.015365471518305442,0.2783024212189956,0.0>,0.05
    ,<0.010792708265868797,0.2981942149140429,0.0>,0.05
    ,<0.006634802775444456,0.3181722398289427,0.0>,0.05
    ,<0.0033906590546878127,0.33830782526723974,0.0>,0.05
    ,<0.0013850437496990401,0.3585894324255118,0.0>,0.05
    ,<0.0007436917714686385,0.3789443579204274,0.0>,0.05
    ,<0.0013940561071900503,0.3992853846073014,0.0>,0.05
    ,<0.0030966098557872394,0.4195563223527741,0.0>,0.05
    ,<0.005503734642989952,0.4397521317217601,0.0>,0.05
    ,<0.008238910501867036,0.45990813004715314,0.0>,0.05
    ,<0.010969237294488877,0.4800705725210666,0.0>,0.05
    ,<0.013454325734648592,0.5002723808907897,0.0>,0.05
    ,<0.01557426566400678,0.5205231644150631,0.0>,0.05
    ,<0.017337073008111322,0.5408143852265284,0.0>,0.05
    ,<0.018863756466897637,0.5611294926684615,0.0>,0.05
    ,<0.020350051221427645,0.58145182341096,0.0>,0.05
    ,<0.022005053065289288,0.6017656848994147,0.0>,0.05
    ,<0.023985806036319387,0.6220561984264469,0.0>,0.05
    ,<0.02635034204527413,0.6423125410747949,0.0>,0.05
    ,<0.029030289950487728,0.6625370014858014,0.0>,0.05
    ,<0.03182225977580789,0.682752436488385,0.0>,0.05
    ,<0.03439695086755055,0.7029992076416139,0.0>,0.05
    ,<0.036325665581574546,0.7233135454492866,0.0>,0.05
    ,<0.03712719648695533,0.7436896634523786,0.0>,0.05
    ,<0.03633003215748625,0.7640398650295758,0.0>,0.05
    ,<0.03353437050087413,0.7841730983750539,0.0>,0.05
    ,<0.028468635318587662,0.8038047042388295,0.0>,0.05
    ,<0.021029621208180724,0.8226001807650867,0.0>,0.05
    ,<0.011293978909322436,0.8402423139969686,0.0>,0.05
    ,<-0.0005045344654708318,0.8565020245837189,0.0>,0.05
    ,<-0.014025138692318497,0.871290591913143,0.0>,0.05
    ,<-0.028881582070899487,0.8846796459121506,0.0>,0.05
    ,<-0.0446960789316536,0.8968845585676419,0.0>,0.05
    ,<-0.061135686209994086,0.9082186898302723,0.0>,0.05
    ,<-0.07792941119236352,0.9190264471233341,0.0>,0.05
    ,<-0.09487972236526931,0.9296060325885771,0.0>,0.05
    ,<-0.1118813911610446,0.9401262538837615,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
