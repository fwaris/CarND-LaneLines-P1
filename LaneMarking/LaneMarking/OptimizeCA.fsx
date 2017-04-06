#r @"C:\Users\family\refs\CALib.dll"
#r @"C:\Users\family\refs\FSharp.Collections.ParallelSeq.dll"
#load "Optimize.fsx"
open System.IO
open Optimize
open CA
open OpenCvSharp
open CAUtils
open System

let tolParms (p:Parm array) =
          {
            LeftLaneSlopes  = (vF p.[0], vF p.[1]) //(0.45,0.72)
            RightLaneSlopes = (vF p.[2], vF p.[3]) //(-0.65,-0.45)
            ColorLow        = Scalar(vF p.[4], vF p.[5],vF p.[6]) //Scalar(90.,90.,190.)
            ColorHigh       = Scalar(vF p.[7], vF p.[8],vF p.[9]) //Scalar(255.,255.,255.)
            CannyTh1        = vF(p.[10]) ///100.
            CannyTh2        = vF(p.[11]) //200.
            CannyAperture   = let v = vI(p.[12]) in if v % 2 = 0 then v + 1 else v //should be odd
            HoughTh         = vI(p.[13]) //15
            HoughMinLineLn  = vF(p.[14]) //30.
            HoughMaxGap     = vF(p.[15]) //100.
        }
    
let CAParms =
          [|
            F(0.45,0.40,0.55)   // LeftlanesSlope 1
            F(0.72,0.55,0.80)   // LeftlanesSlope 2
            F(-0.65,-0.70,-0.55) // RightLaneSlopes 1
            F(-0.45,-0.55,-0.40) // RightLaneSlopes 2
            F(90.,70.,120.)     // ColorLow 1
            F(90.,70.,120.)     // ColorLow 2
            F(190.,175.,205.)   // ColorLow 3
            F(255.,250.,255.)   // ColorHigh 1
            F(255.,250.,255.)   // ColorHigh 2
            F(255.,250.,255.)   // ColorHigh 3
            F(100.,40.,150.)    // CannyTh1
            F(200.,150.,300.)   // CannyTh2
            I(3,2,7)            // CannyAperture
            I(15,10,30)         //HoughTh
            F(30.,10.,50.)      // HoughMinLineLn  = vF(p.[14]) //30.
            F(100.,75.,150.)    // HoughMaxGap     = vF(p.[15]) //100.
          |]

type Label = {Img:Mat; SegLeft:LineSegmentPoint; SegRight:LineSegmentPoint}

let readLabels file =
     File.ReadAllLines(file)
     |> Seq.map (fun l -> l.Split([|'\t'|]))
     |> Seq.map (fun ls ->
        {
            Img = new Mat(ls.[0],ImreadModes.AnyColor)
            SegLeft = new LineSegmentPoint(Point(int ls.[1], int ls.[2]),Point(int ls.[3], int ls.[4]))
            SegRight = new LineSegmentPoint(Point(int ls.[5], int ls.[6]),Point(int ls.[7], int ls.[8]))
        })
    |> Seq.toArray

let labels = readLabels @"C:\Users\family\CarND-LaneLines-P1\optframeLabels\lablesCorrected.txt"
let (mask,_) = makeMask (labels.[0].Img.Width,labels.[0].Img.Height)

let findSlopes parms mask mImage =
    use edges = markEdges parms mImage mask
    let segStd = Cv2.HoughLinesP(!>edges,1.,Math.PI/180.,parms.HoughTh,parms.HoughMinLineLn,parms.HoughMaxGap)
    let h = mImage.Height
    let (sL,sR) = segmentLanes h parms segStd
    let segLeft,segRight = laneOverlay h parms sL, laneOverlay h parms sR
    segLeft,segRight

let dist (L1:LineSegmentPoint) (L2:LineSegmentPoint) = 
    abs(L1.P1.X - L2.P1.X)
    + abs(L1.P1.Y - L2.P1.Y)
    + abs(L1.P2.X - L2.P2.X)
    + abs(L1.P2.Y - L2.P2.Y)
    |> float
   
let fitness parms =
    let lparms = tolParms parms // parms
    (0.,labels) ||> Array.fold(fun acc lbl ->
        let ls,rs = findSlopes lparms mask lbl.Img
        (dist ls lbl.SegLeft) + (dist rs lbl.SegRight) + acc
        )

let defaultNetwork = CAUtils.hexagonNetwork

let inline makeCA fitness comparator pop bspace kd influence =
        {
            Population           = pop
            Network              = defaultNetwork
            KnowlegeDistribution = kd
            BeliefSpace          = bspace
            Acceptance           = CARunner.acceptance 10 comparator
            Influence            = influence
            Update               = CARunner.update
            Fitness              = fitness
            Comparator           = comparator
        }

let ipdKdist      c p    = KDIPDGame.knowledgeDist c p
let inline bsp fitness parms comparator = CARunner.defaultBeliefSpace parms comparator fitness
let inline createPop bsp parms init = CAUtils.createPop (init bsp) parms 100 false
let createKdIpdCA vmx f c p  = 
    let bsp = bsp f p c
    let pop = createPop bsp p CAUtils.baseKsInit |> KDIPDGame.initKS
    let kd = ipdKdist vmx c pop 
    makeCA f c pop bsp kd KDIPDGame.ipdInfluence
let comparator  = CAUtils.Minimize
let kdIpdCA         = createKdIpdCA  (0.2, 0.9) fitness comparator CAParms 
let termination step = step.Count > 100
let best stp = if stp.Best.Length > 0 then stp.Best.[0].Fitness,stp.Best.[0].Parms else 0.0,[||]
let runCollect data maxBest ca =
    let loop stp = 
        let stp = CARunner.step stp maxBest
        printfn "step %i. fitness=%A" stp.Count (best stp)
//        printfn "KS = %A" (stp.CA.Population |> Seq.countBy (fun x->x.KS))
        stp
    let step = {CA=ca; Best=[]; Count=0; Progress=[]}
    step 
    |> Seq.unfold (fun s -> let s = loop s in (data s,s)  |> Some ) 
let ipdDataCollector s = 
    best s, 
    s.CA.Population 
    |> Seq.collect (fun p-> 
        let ({KDIPDGame.KS=ks;KDIPDGame.Level=lvl},m) = p.KS
        ([ks,lvl],m) ||> Map.fold (fun acc k v -> (k,v)::acc)
        ) 
    |> Seq.groupBy fst
    |> Seq.map (fun (k,vs) -> k, vs |> Seq.map snd |> Seq.sum)
    |> Seq.sortBy fst
    |> Seq.toList
let tk s = s |> Seq.truncate 250 |> Seq.toList
let kdIpd           = kdIpdCA |> runCollect ipdDataCollector 2 |> tk
