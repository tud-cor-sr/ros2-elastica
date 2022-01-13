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
    ,<0.14735981285239977,0.15522722889938417,0.0>,0.05
    ,<0.1285108709867781,0.16191140223107264,0.0>,0.05
    ,<0.10965566883579089,0.16857220460400274,0.0>,0.05
    ,<0.09080565135855348,0.17524168051070677,0.0>,0.05
    ,<0.07197858530316956,0.18196886408572582,0.0>,0.05
    ,<0.053198493825782775,0.18881784907591917,0.0>,0.05
    ,<0.03449598032787781,0.19586567066336555,0.0>,0.05
    ,<0.015908947649241188,0.20319995955552944,0.0>,0.05
    ,<-0.002516325679464511,0.21091632484601286,0.0>,0.05
    ,<-0.020723854283044584,0.2191154260440381,0.0>,0.05
    ,<-0.0386465015731624,0.22789969522415343,0.0>,0.05
    ,<-0.05620461835281548,0.2373696898539056,0.0>,0.05
    ,<-0.07330498612495527,0.2476200829344774,0.0>,0.05
    ,<-0.089840316370907,0.2587353497811295,0.0>,0.05
    ,<-0.10568954779034277,0.2707852711990214,0.0>,0.05
    ,<-0.12071916132618896,0.2838204536142469,0.0>,0.05
    ,<-0.13478566277099807,0.2978681396198168,0.0>,0.05
    ,<-0.14773928859795088,0.31292864987287095,0.0>,0.05
    ,<-0.15942885690510833,0.3289728265307876,0.0>,0.05
    ,<-0.1697075402243075,0.34594083755172367,0.0>,0.05
    ,<-0.17843919073406536,0.3637426295579924,0.0>,0.05
    ,<-0.1855047310511119,0.3822601910671587,0.0>,0.05
    ,<-0.19080805944044746,0.4013516175859658,0.0>,0.05
    ,<-0.19428092174572076,0.42085677503748076,0.0>,0.05
    ,<-0.19588628570487918,0.44060417811835495,0.0>,0.05
    ,<-0.1956199011330664,0.46041855258248804,0.0>,0.05
    ,<-0.19350992630617175,0.48012848003836445,0.0>,0.05
    ,<-0.18961470958444682,0.4995735207946696,0.0>,0.05
    ,<-0.18401900517255865,0.5186102973347955,0.0>,0.05
    ,<-0.17682904329059732,0.5371171562809124,0.0>,0.05
    ,<-0.16816694848455163,0.5549972069159044,0.0>,0.05
    ,<-0.15816500497926567,0.5721797092112859,0.0>,0.05
    ,<-0.14696020965297166,0.5886199423142802,0.0>,0.05
    ,<-0.13468945377356167,0.6042977973661713,0.0>,0.05
    ,<-0.12148555388567031,0.6192154019783526,0.0>,0.05
    ,<-0.10747423071002642,0.6333941021969853,0.0>,0.05
    ,<-0.09277203265867268,0.6468711023675107,0.0>,0.05
    ,<-0.0774851182909109,0.659696020103485,0.0>,0.05
    ,<-0.06170876596099048,0.6719275481990499,0.0>,0.05
    ,<-0.0455274486719311,0.683630359720445,0.0>,0.05
    ,<-0.029015315741474482,0.6948723309086307,0.0>,0.05
    ,<-0.012236926662035472,0.7057221184910919,0.0>,0.05
    ,<0.0047518879372542315,0.7162470883738529,0.0>,0.05
    ,<0.02190314785929101,0.7265115780299773,0.0>,0.05
    ,<0.03917590126290721,0.7365754557126427,0.0>,0.05
    ,<0.056535519217174916,0.7464929412796366,0.0>,0.05
    ,<0.07395313465743844,0.756311645872616,0.0>,0.05
    ,<0.09140524641814668,0.7660717964599978,0.0>,0.05
    ,<0.10887348208391447,0.7758056073254024,0.0>,0.05
    ,<0.12634451234549007,0.7855367700268355,0.0>,0.05
    ,<0.14381008716248356,0.7952800284049232,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
