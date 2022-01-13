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
    ,<0.06215832127251217,0.007718420415257124,0.0>,0.05
    ,<0.05163446414337454,0.024725055178852803,0.0>,0.05
    ,<0.04110271753695497,0.04172547736752783,0.0>,0.05
    ,<0.03057510833288706,0.05872711397776266,0.0>,0.05
    ,<0.020070062792903152,0.07574129564511083,0.0>,0.05
    ,<0.009611806698710715,0.0927827674100093,0.0>,0.05
    ,<-0.0007702007421869924,0.10986913451118382,0.0>,0.05
    ,<-0.011041798595457153,0.12702023838605875,0.0>,0.05
    ,<-0.021164625960601608,0.14425746457013786,0.0>,0.05
    ,<-0.031096602856519794,0.16160298985590021,0.0>,0.05
    ,<-0.04079240313849737,0.17907898105612996,0.0>,0.05
    ,<-0.05020393511719568,0.19670676214533533,0.0>,0.05
    ,<-0.059280846947967966,0.21450597052508602,0.0>,0.05
    ,<-0.06797107364444167,0.23249372667493756,0.0>,0.05
    ,<-0.07622144059033904,0.2506838444244827,0.0>,0.05
    ,<-0.08397833463090426,0.2690861113059422,0.0>,0.05
    ,<-0.09118844832272732,0.28770566963205974,0.0>,0.05
    ,<-0.09779959597171388,0.30654252873452303,0.0>,0.05
    ,<-0.1037615921371868,0.32559123684346464,0.0>,0.05
    ,<-0.1090271749354957,0.34484073710217633,0.0>,0.05
    ,<-0.1135529484843029,0.36427442605181815,0.0>,0.05
    ,<-0.1173003120081059,0.3838704246595031,0.0>,0.05
    ,<-0.12023633828080568,0.40360206192228404,0.0>,0.05
    ,<-0.12233456190233148,0.4234385598549633,0.0>,0.05
    ,<-0.12357563885937178,0.4433458970793309,0.0>,0.05
    ,<-0.12394784306464818,0.4632878172617518,0.0>,0.05
    ,<-0.12344737289963505,0.48322693930429045,0.0>,0.05
    ,<-0.12207845063471898,0.5031259194175973,0.0>,0.05
    ,<-0.11985320908680476,0.5229486116782467,0.0>,0.05
    ,<-0.11679137190817496,0.5426611737859081,0.0>,0.05
    ,<-0.11291974532093547,0.5622330684621399,0.0>,0.05
    ,<-0.1082715488371997,0.5816379178884923,0.0>,0.05
    ,<-0.10288561967176593,0.6008541780418373,0.0>,0.05
    ,<-0.09680552961180437,0.619865610822173,0.0>,0.05
    ,<-0.09007865387563549,0.6386615434559778,0.0>,0.05
    ,<-0.08275522914608581,0.6572369158297122,0.0>,0.05
    ,<-0.07488743298661221,0.6755921263465042,0.0>,0.05
    ,<-0.06652850993707138,0.6937326950311716,0.0>,0.05
    ,<-0.057731961549047875,0.7116687686233879,0.0>,0.05
    ,<-0.04855080927332404,0.7294144962413258,0.0>,0.05
    ,<-0.03903693119166023,0.746987306022937,0.0>,0.05
    ,<-0.029240466692940018,0.7644071132497984,0.0>,0.05
    ,<-0.019209277759619784,0.7816954891909944,0.0>,0.05
    ,<-0.008988451812103384,0.7988748176201017,0.0>,0.05
    ,<0.0013801708556611403,0.8159674629568107,0.0>,0.05
    ,<0.01185846217195642,0.8329949704581326,0.0>,0.05
    ,<0.02241247959992887,0.8499773149317918,0.0>,0.05
    ,<0.03301296055859783,0.8669322100461141,0.0>,0.05
    ,<0.04363584698966305,0.8838744853854062,0.0>,0.05
    ,<0.054262844493565526,0.9008155327968079,0.0>,0.05
    ,<0.06488201537847331,0.917762817155469,0.0>,0.05
    texture{
        pigment{ color rgb<0.45,0.39,1> transmit 0.000000 }
        finish{ phong 1 }
    }
    }
