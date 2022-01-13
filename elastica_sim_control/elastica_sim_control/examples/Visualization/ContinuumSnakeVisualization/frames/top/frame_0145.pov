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
    ,<0.14136128356072172,0.1333964195039788,0.0>,0.05
    ,<0.12319587087071121,0.14176178041858617,0.0>,0.05
    ,<0.10502324100347808,0.15010693270291428,0.0>,0.05
    ,<0.086855995808785,0.15845909736805647,0.0>,0.05
    ,<0.06871366768984746,0.16686002355363044,0.0>,0.05
    ,<0.050622478924031135,0.17536436787402984,0.0>,0.05
    ,<0.03261539344282149,0.18403789979458712,0.0>,0.05
    ,<0.014732472716766273,0.1929554995949919,0.0>,0.05
    ,<-0.0029784952823478737,0.20219892059076658,0.0>,0.05
    ,<-0.02046116371139143,0.2118542901425817,0.0>,0.05
    ,<-0.03764969955525428,0.22200933078274218,0.0>,0.05
    ,<-0.05446790167170305,0.2327502943863566,0.0>,0.05
    ,<-0.07082853221745326,0.24415862495875196,0.0>,0.05
    ,<-0.08663304397028204,0.2563073979295544,0.0>,0.05
    ,<-0.10177188852465929,0.2692576290792259,0.0>,0.05
    ,<-0.11612557056617609,0.2830545976669591,0.0>,0.05
    ,<-0.12956657294266286,0.2977243835543599,0.0>,0.05
    ,<-0.1419622095420519,0.3132708640832513,0.0>,0.05
    ,<-0.15317837475036525,0.329673445938618,0.0>,0.05
    ,<-0.16308405295965647,0.34688580565182336,0.0>,0.05
    ,<-0.17155634444687534,0.3648358747702372,0.0>,0.05
    ,<-0.17848566900639623,0.38342722666078954,0.0>,0.05
    ,<-0.18378074448883827,0.40254190794675476,0.0>,0.05
    ,<-0.18737291818161894,0.42204462050351865,0.0>,0.05
    ,<-0.18921946299549525,0.44178801864342826,0.0>,0.05
    ,<-0.18930553758323215,0.46161876395214707,0.0>,0.05
    ,<-0.1876446385002652,0.48138389546994054,0.0>,0.05
    ,<-0.18427752517483775,0.5009370427516304,0.0>,0.05
    ,<-0.1792697510178973,0.520144035328286,0.0>,0.05
    ,<-0.17270806366205588,0.5388875419018474,0.0>,0.05
    ,<-0.16469602641008801,0.5570704887706522,0.0>,0.05
    ,<-0.15534925174007483,0.5746181415869435,0.0>,0.05
    ,<-0.14479062653304034,0.5914788656559097,0.0>,0.05
    ,<-0.1331458559266848,0.6076236897539281,0.0>,0.05
    ,<-0.1205395717020388,0.6230448770409563,0.0>,0.05
    ,<-0.10709215880918237,0.6377537471421266,0.0>,0.05
    ,<-0.09291736224039363,0.651778002206728,0.0>,0.05
    ,<-0.0781206596830025,0.6651587889402718,0.0>,0.05
    ,<-0.06279832536414127,0.6779476929358422,0.0>,0.05
    ,<-0.04703707388308245,0.690203814491552,0.0>,0.05
    ,<-0.030914152540981517,0.7019910303058438,0.0>,0.05
    ,<-0.01449774936918146,0.7133755023697512,0.0>,0.05
    ,<0.002152409104721993,0.72442346304684,0.0>,0.05
    ,<0.01898437868561919,0.7351992790364343,0.0>,0.05
    ,<0.03595332718859194,0.7457637816019541,0.0>,0.05
    ,<0.053020940413139726,0.7561728395971619,0.0>,0.05
    ,<0.07015495272610353,0.7664761483833344,0.0>,0.05
    ,<0.087328829541833,0.7767162053608561,0.0>,0.05
    ,<0.10452161277480491,0.7869274439877498,0.0>,0.05
    ,<0.12171792297673303,0.7971354975742505,0.0>,0.05
    ,<0.13890809896746772,0.8073565644847779,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
