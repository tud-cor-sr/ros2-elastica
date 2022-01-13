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
    ,<-0.0193121814440179,0.35376195487708145,0.0>,0.05
    ,<-0.03302004849464902,0.3392095837871166,0.0>,0.05
    ,<-0.046903829187406834,0.3248465939617004,0.0>,0.05
    ,<-0.061238404546528895,0.3109540113865954,0.0>,0.05
    ,<-0.07634256163783425,0.2979207245080712,0.0>,0.05
    ,<-0.09250311380876293,0.2862383048266411,0.0>,0.05
    ,<-0.10989322856544406,0.2764906972709168,0.0>,0.05
    ,<-0.1284945482442862,0.2693224020712362,0.0>,0.05
    ,<-0.1480383996885609,0.26537495878734074,0.0>,0.05
    ,<-0.16798482738857604,0.265193990061519,0.0>,0.05
    ,<-0.1875548530142179,0.2691241778054793,0.0>,0.05
    ,<-0.20581904144293825,0.27722097122092404,0.0>,0.05
    ,<-0.22182714458093375,0.2892088536933348,0.0>,0.05
    ,<-0.23474728591278296,0.3045037933389322,0.0>,0.05
    ,<-0.24397761052230577,0.3222961213734899,0.0>,0.05
    ,<-0.2492023697950645,0.34166917461736324,0.0>,0.05
    ,<-0.25038456537067477,0.3617182843973552,0.0>,0.05
    ,<-0.24771353818963768,0.3816395906397982,0.0>,0.05
    ,<-0.24153407878428362,0.40077861112763596,0.0>,0.05
    ,<-0.23227704605219063,0.41864360006326173,0.0>,0.05
    ,<-0.2204032917517156,0.4348948661095046,0.0>,0.05
    ,<-0.20636492715012625,0.44932201849788433,0.0>,0.05
    ,<-0.19058261254228598,0.4618187996424123,0.0>,0.05
    ,<-0.17343479506581597,0.4723617560281421,0.0>,0.05
    ,<-0.15525355302682028,0.48099459440977554,0.0>,0.05
    ,<-0.13632329922216924,0.48781768858712715,0.0>,0.05
    ,<-0.1168806074179187,0.4929821830671928,0.0>,0.05
    ,<-0.09711435231537016,0.49668799534489555,0.0>,0.05
    ,<-0.07716625746401536,0.49918512451590996,0.0>,0.05
    ,<-0.05713291259193572,0.5007779564803662,0.0>,0.05
    ,<-0.03707143767422375,0.5018324154003478,0.0>,0.05
    ,<-0.01701219124065663,0.5027816620211316,0.0>,0.05
    ,<0.003016965967487114,0.5041251132585898,0.0>,0.05
    ,<0.0229519913433634,0.5064182948937354,0.0>,0.05
    ,<0.04264146942569326,0.5102482042366612,0.0>,0.05
    ,<0.061792011506707246,0.5161863643847835,0.0>,0.05
    ,<0.07992858345820666,0.5247134115332612,0.0>,0.05
    ,<0.09639628056216183,0.536119053012929,0.0>,0.05
    ,<0.11042516297782916,0.5504053732456309,0.0>,0.05
    ,<0.12125431248106307,0.5672371437724604,0.0>,0.05
    ,<0.1282839968196532,0.5859694309520321,0.0>,0.05
    ,<0.13120582785255783,0.6057583204932762,0.0>,0.05
    ,<0.13006428719463917,0.625726946612641,0.0>,0.05
    ,<0.1252303872945305,0.6451349176374521,0.0>,0.05
    ,<0.11730475758488336,0.6634993955833073,0.0>,0.05
    ,<0.10699224850839016,0.6806391279990686,0.0>,0.05
    ,<0.09499120298305509,0.6966431698949902,0.0>,0.05
    ,<0.08192136797563714,0.7117871805176078,0.0>,0.05
    ,<0.06828857551749669,0.7264256022355076,0.0>,0.05
    ,<0.05446521645208084,0.740882499444193,0.0>,0.05
    ,<0.040660519590317974,0.7553551892011457,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
