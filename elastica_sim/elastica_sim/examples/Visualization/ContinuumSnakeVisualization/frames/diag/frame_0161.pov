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
    ,<0.10173827746126254,0.05217561660842033,0.0>,0.05
    ,<0.08750569938273009,0.06622551724222288,0.0>,0.05
    ,<0.07326454872952896,0.08026433105615167,0.0>,0.05
    ,<0.05902851977211809,0.09430589455513542,0.0>,0.05
    ,<0.044818743042545114,0.10837144594427799,0.0>,0.05
    ,<0.030663224520579877,0.12248878001497748,0.0>,0.05
    ,<0.016596378074473443,0.13669129156821977,0.0>,0.05
    ,<0.002658654142218426,0.1510168936452878,0.0>,0.05
    ,<-0.011103746096640055,0.16550680557158806,0.0>,0.05
    ,<-0.024639094674461258,0.18020421338908513,0.0>,0.05
    ,<-0.0378903342743315,0.19515281270930032,0.0>,0.05
    ,<-0.05079529405307211,0.21039525171007722,0.0>,0.05
    ,<-0.06328697555905252,0.22597150082473702,0.0>,0.05
    ,<-0.07529396336033638,0.24191718558615816,0.0>,0.05
    ,<-0.0867410133990838,0.2582619302461899,0.0>,0.05
    ,<-0.09754986502947807,0.2750277712601394,0.0>,0.05
    ,<-0.10764030995069926,0.2922277105596752,0.0>,0.05
    ,<-0.11693153271645919,0.3098644868558161,0.0>,0.05
    ,<-0.1253437140464824,0.32792964723725204,0.0>,0.05
    ,<-0.13279986124360024,0.3464029989707432,0.0>,0.05
    ,<-0.13922780205035928,0.3652525111854937,0.0>,0.05
    ,<-0.14456225230102257,0.3844347172186226,0.0>,0.05
    ,<-0.14874684712951283,0.40389564116187265,0.0>,0.05
    ,<-0.15173601358412678,0.4235722383658305,0.0>,0.05
    ,<-0.15349656176635165,0.44339430221420834,0.0>,0.05
    ,<-0.1540088833845326,0.4632867525680386,0.0>,0.05
    ,<-0.1532676704440523,0.48317218910285764,0.0>,0.05
    ,<-0.15128210057876917,0.5029735697305624,0.0>,0.05
    ,<-0.14807547553015737,0.5226168632208323,0.0>,0.05
    ,<-0.14368434082869608,0.5420335278445455,0.0>,0.05
    ,<-0.1381571528941188,0.5611626836468004,0.0>,0.05
    ,<-0.13155259015346066,0.5799528729440868,0.0>,0.05
    ,<-0.12393762429199343,0.5983633379421673,0.0>,0.05
    ,<-0.1153854748864197,0.6163647819224191,0.0>,0.05
    ,<-0.1059735657748478,0.6339396166914777,0.0>,0.05
    ,<-0.09578158634531876,0.6510817304549912,0.0>,0.05
    ,<-0.08488973839243938,0.6677958343202133,0.0>,0.05
    ,<-0.07337722260675751,0.6840964611505682,0.0>,0.05
    ,<-0.0613209914059218,0.7000066975737839,0.0>,0.05
    ,<-0.048794769469866654,0.7155567296567572,0.0>,0.05
    ,<-0.0358683219638645,0.7307822769333949,0.0>,0.05
    ,<-0.02260693428850981,0.7457229799434276,0.0>,0.05
    ,<-0.00907105658687214,0.7604207952487417,0.0>,0.05
    ,<0.004683938842112486,0.7749184403324387,0.0>,0.05
    ,<0.01860793939446236,0.7892579201240871,0.0>,0.05
    ,<0.03265626156016133,0.8034791573666344,0.0>,0.05
    ,<0.046789849189055925,0.8176187409903715,0.0>,0.05
    ,<0.06097553244882484,0.8317087994495401,0.0>,0.05
    ,<0.07518636836154753,0.8457759993921745,0.0>,0.05
    ,<0.08940207173646005,0.8598406630686691,0.0>,0.05
    ,<0.10360953298681014,0.8739159902788475,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }