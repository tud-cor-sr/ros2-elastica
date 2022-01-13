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
    ,<-0.037992968001361836,0.43845112645898954,0.0>,0.05
    ,<-0.04008075699476492,0.41856494979516595,0.0>,0.05
    ,<-0.04241645904574314,0.3987152699377695,0.0>,0.05
    ,<-0.04540266830035777,0.37896130703482644,0.0>,0.05
    ,<-0.04954827809621599,0.35942512862084885,0.0>,0.05
    ,<-0.05540278648150625,0.34033753620230356,0.0>,0.05
    ,<-0.06348087098802944,0.3220844106759106,0.0>,0.05
    ,<-0.07417385636086107,0.3052328609367918,0.0>,0.05
    ,<-0.08765388972820501,0.29051711138299685,0.0>,0.05
    ,<-0.10378880074207163,0.27877175757272044,0.0>,0.05
    ,<-0.12209429910200907,0.27081541748425586,0.0>,0.05
    ,<-0.14174834402002426,0.2673073014390913,0.0>,0.05
    ,<-0.16167727545993074,0.2686145456828843,0.0>,0.05
    ,<-0.1806992495161056,0.2747302380024774,0.0>,0.05
    ,<-0.19768874519293003,0.28526729001565915,0.0>,0.05
    ,<-0.2117177001826041,0.29952720073809935,0.0>,0.05
    ,<-0.2221389519324328,0.31661708749538087,0.0>,0.05
    ,<-0.22860545115595407,0.3355739996700878,0.0>,0.05
    ,<-0.2310410006981005,0.35546734559910415,0.0>,0.05
    ,<-0.22958403062973115,0.375467826784121,0.0>,0.05
    ,<-0.22452383840001824,0.3948832343246522,0.0>,0.05
    ,<-0.21624253442146085,0.413168601725536,0.0>,0.05
    ,<-0.20516911329512755,0.42992063143115666,0.0>,0.05
    ,<-0.19174659329251764,0.4448654397289593,0.0>,0.05
    ,<-0.17640895293905656,0.4578451162224467,0.0>,0.05
    ,<-0.15956442384259498,0.4688053238592186,0.0>,0.05
    ,<-0.14158315775691555,0.47778505579444747,0.0>,0.05
    ,<-0.12278791554995404,0.4849089419910922,0.0>,0.05
    ,<-0.10344725390463436,0.4903821679308562,0.0>,0.05
    ,<-0.08377168677691693,0.49448808296472924,0.0>,0.05
    ,<-0.06391445369959155,0.4975886502331734,0.0>,0.05
    ,<-0.04397945938483104,0.5001238599812806,0.0>,0.05
    ,<-0.02404073631021526,0.5026055543610604,0.0>,0.05
    ,<-0.004177023467548629,0.5056044108598041,0.0>,0.05
    ,<0.015479496178525889,0.5097263076411513,0.0>,0.05
    ,<0.03468786171782525,0.51557123611985,0.0>,0.05
    ,<0.05305246557874736,0.5236678916929437,0.0>,0.05
    ,<0.0700109674907537,0.5343843124272112,0.0>,0.05
    ,<0.08487783313813989,0.547836511178341,0.0>,0.05
    ,<0.0969446511403991,0.5638330292654365,0.0>,0.05
    ,<0.10561440473983846,0.581884046274151,0.0>,0.05
    ,<0.11052732533811227,0.6012854059736421,0.0>,0.05
    ,<0.11163451865015946,0.6212590160116604,0.0>,0.05
    ,<0.1091958992538542,0.6411070411429441,0.0>,0.05
    ,<0.10371038163733881,0.6603326457564831,0.0>,0.05
    ,<0.09581076318771145,0.6786964323572705,0.0>,0.05
    ,<0.08616089095281866,0.6962037350091926,0.0>,0.05
    ,<0.07537909573501186,0.7130389686768115,0.0>,0.05
    ,<0.06399005909086383,0.7294719161918275,0.0>,0.05
    ,<0.052389492134949124,0.745759183722353,0.0>,0.05
    ,<0.040800114672845944,0.7620575209257482,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
