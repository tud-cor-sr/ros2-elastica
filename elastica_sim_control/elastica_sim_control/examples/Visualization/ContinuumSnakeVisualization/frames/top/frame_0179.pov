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
    ,<0.05464982475252155,0.001523821101679561,0.0>,0.05
    ,<0.044824549947357295,0.01894345378334939,0.0>,0.05
    ,<0.03499167465746808,0.0363576362703265,0.0>,0.05
    ,<0.025162702937672605,0.05377284420778299,0.0>,0.05
    ,<0.01535523882175318,0.07119894597371139,0.0>,0.05
    ,<0.005592398148082942,0.08864877081871843,0.0>,0.05
    ,<-0.004097752397756463,0.10613761982637869,0.0>,0.05
    ,<-0.013682728439270904,0.12368271590143128,0.0>,0.05
    ,<-0.023126136754047975,0.14130259489872204,0.0>,0.05
    ,<-0.032388166958187525,0.15901644512097732,0.0>,0.05
    ,<-0.041426076188999066,0.1768434068468855,0.0>,0.05
    ,<-0.050194679045795595,0.19480184739545917,0.0>,0.05
    ,<-0.05864685601373284,0.21290863055150175,0.0>,0.05
    ,<-0.06673409324294163,0.23117840196826225,0.0>,0.05
    ,<-0.07440706478095446,0.24962291435743847,0.0>,0.05
    ,<-0.08161626515259646,0.2682504177241698,0.0>,0.05
    ,<-0.08831269565945557,0.28706514039878034,0.0>,0.05
    ,<-0.09444860217060129,0.3060668859143839,0.0>,0.05
    ,<-0.09997825587085017,0.3252507686482073,0.0>,0.05
    ,<-0.10485876191582337,0.34460710741169487,0.0>,0.05
    ,<-0.10905087479758249,0.36412149078038175,0.0>,0.05
    ,<-0.11251979407534275,0.3837750209993268,0.0>,0.05
    ,<-0.11523591057162735,0.40354473506811295,0.0>,0.05
    ,<-0.11717547167559976,0.42340419257867423,0.0>,0.05
    ,<-0.11832113535770092,0.4433242106833836,0.0>,0.05
    ,<-0.11866238597635358,0.4632737179522634,0.0>,0.05
    ,<-0.11819579077299164,0.4832206915908113,0.0>,0.05
    ,<-0.11692508366593984,0.5031331372218819,0.0>,0.05
    ,<-0.11486107190002812,0.5229800676928474,0.0>,0.05
    ,<-0.1120213704708762,0.5427324374347642,0.0>,0.05
    ,<-0.10842997815183991,0.5623639917602304,0.0>,0.05
    ,<-0.10411671659542882,0.581851995859941,0.0>,0.05
    ,<-0.0991165597017953,0.6011778156179018,0.0>,0.05
    ,<-0.09346888381427897,0.6203273310267325,0.0>,0.05
    ,<-0.08721667015332708,0.639291172184457,0.0>,0.05
    ,<-0.08040568933799665,0.6580647768592666,0.0>,0.05
    ,<-0.07308369420420044,0.6766482767851498,0.0>,0.05
    ,<-0.06529964191552906,0.6950462267338622,0.0>,0.05
    ,<-0.057102960178285374,0.7132671957181473,0.0>,0.05
    ,<-0.048542865840187285,0.7313232433281587,0.0>,0.05
    ,<-0.039667737852685465,0.7492293062531126,0.0>,0.05
    ,<-0.03052454099803998,0.7670025206724571,0.0>,0.05
    ,<-0.02115829229210152,0.7846615056508293,0.0>,0.05
    ,<-0.011611558806959748,0.8022256311832022,0.0>,0.05
    ,<-0.0019239739203115514,0.8197142923196374,0.0>,0.05
    ,<0.00786824131978705,0.8371462079998216,0.0>,0.05
    ,<0.01773276403217959,0.8545387599173494,0.0>,0.05
    ,<0.027641672733639832,0.8719073829095563,0.0>,0.05
    ,<0.0375719751285284,0.8892650139707728,0.0>,0.05
    ,<0.047506163943066795,0.9066216019154184,0.0>,0.05
    ,<0.05743280020053893,0.9239836738690992,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
