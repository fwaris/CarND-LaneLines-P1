#I @"..\packages\OpenCvSharp3-AnyCPU.3.2.0.20170324\lib\net40"
#r @"OpenCvSharp.dll"
#r @"OpenCvSharp.Blob.dll"
#r @"OpenCvSharp.Extensions.dll"
#r @"OpenCvSharp.UserInterface.dll"

System.Environment.CurrentDirectory <- __SOURCE_DIRECTORY__ + @"..\..\packages\OpenCvSharp3-AnyCPU.3.2.0.20170324\NativeDlls\x64"
let uiCtx = System.Threading.SynchronizationContext.Current
open OpenCvSharp
open System

let win t i = 
    async{
        do! Async.SwitchToContext uiCtx
        new Window((t:string), WindowMode.AutoSize,i) |> ignore 
        } |> Async.Start

let inline (!>) (x:^a) : ^b = ((^a or ^b) : (static member op_Implicit : ^a -> ^b) x)

//let leftLaneSlopes = (0.45,0.72)
//let rightLaneSlopes = (-0.65,-0.45)

type LParms =
    {
        LeftLaneSlopes  : float * float
        RightLaneSlopes : float * float
        ColorLow        : Scalar
        ColorHigh       : Scalar
        CannyTh1        : float
        CannyTh2        : float
        CannyAperture   : int
        HoughTh         : int
        HoughMinLineLn  : float
        HoughMaxGap     : float
    }
    static member Default =
        {
            LeftLaneSlopes  = (0.45,0.72)
            RightLaneSlopes = (-0.65,-0.45)
            ColorLow        = Scalar(90.,90.,190.)
            ColorHigh       = Scalar(255.,255.,255.)
            CannyTh1        = 100.
            CannyTh2        = 200.
            CannyAperture   = 3
            HoughTh         = 15
            HoughMinLineLn  = 30.
            HoughMaxGap     = 100.
        }

let toWorldP h (p:Point) = Point(p.X, h - p.Y)
let toScreenP h (p:Point) = Point(p.X, h - p.Y)

let slope h (l:LineSegmentPoint) =
    let p1 = toWorldP h l.P1
    let p2 = toWorldP h l.P2
    let s = if p1.X=p2.X then Math.PI/2. else Math.Atan(float(p1.Y - p2.Y) / float(p1.X - p2.X)) 
//    printfn "slope:[%A,%A] %A,%A=%f" l.P1 l.P2 p1 p2 s
    s

let makeMask (w,h) = 
    let baseL = float w * 0.1 |> int
    let baseR = float w * 0.95 |> int
    let maskPts =
        [|
            baseL,h
            baseR,h
            w/2 + 40, float h * 0.60 |> int
            w/2 - 40, float h * 0.60 |> int
        |]
        |> Array.map Point
        |> InputArray.Create
    let mask = new Mat(Size(w,h), (MatType.CV_8UC1 : MatType))
    Cv2.FillConvexPoly(!>mask,maskPts,Scalar.White)
    mask,maskPts

let isolateLaneMarkers parms (mImage:Mat)=
    let m = new Mat()
    Cv2.InRange(!>mImage,parms.ColorLow,parms.ColorHigh,!>m)
    m

//canny edge detection
let markEdges parms (mImage:Mat) (mask:Mat) =
    let gray = isolateLaneMarkers parms mImage
    Cv2.BitwiseAnd(!>gray,!>mask,!>gray)
    Cv2.Canny(!>gray,!>gray,parms.CannyTh1,parms.CannyTh2, parms.CannyAperture)
    gray

//slopes for left and right lanes in radians
//line segments with slopes outside these ranges
//are discarded. Note these would have
//to be adjusted according to 
//camera parameters and mounting 
//let leftLaneSlopes = (0.45,0.72)
//let rightLaneSlopes = (-0.65,-0.45)


let (<->) s (a,b)  = s >= a && s <= b //between operator

//classify the given line segments
//into left and right lane buckets
//uses leftLane and rightLane slopes above
//returns the slope (in radians) along with 
//segmented lane segments
//discards segments not in the two ranges
let segmentLanes h parms (segStds:LineSegmentPoint[]) =
    (([],[]),segStds) ||> Array.fold(fun (ls,rs) seg ->
        let s = slope h seg
        if s <-> parms.RightLaneSlopes then 
            (ls,(s,seg)::rs)
        elif s <-> parms.LeftLaneSlopes then
            ((s,seg)::ls,rs)
        else
            (ls,rs)
        )

//'null' line segment used when no line segments found in the image
let NullSeg = LineSegmentPoint(Point(0,0),Point(0,0))

//Returns a single line segment
//from a given collection of line segments (with slopes)
//It returns the 'average' of the line segments
//Both, the slopes of all the line segments
//and the points are averaged.
//These averaged values (slope and point)
//are used to derive a line equation
//The returned segment represents a portion
//of this line.
let laneOverlay h parms (segs:(float*LineSegmentPoint) seq) =
    if segs |> Seq.isEmpty then 
        NullSeg
    else
        let avgTheta = segs |> Seq.averageBy fst
        let pxI = segs |> Seq.collect (fun (_,s) -> [s.P1.X;s.P2.X]) |> Seq.map float |> Seq.average |> int
        let pyI = segs |> Seq.collect (fun (_,s) -> [s.P1.Y;s.P2.Y]) |> Seq.map float |> Seq.average |> int
        let pW = toWorldP h (Point(pxI,pyI)) //fseg.P2
        let (px,py) = float pW.X, float pW.Y
        let slope = Math.Tan(avgTheta) 
        //(y - y1) = m(x - x1) ==> (y - y1) = Tan(theta) (x - x1) ==> x = (y - y1)/(Tan(theta)) + x1
        let y x' = (slope * (x' - px) ) - py // y given x - not used
        let x y' = ((y' - py)/slope) + px    // x given y
        let highMark = 200
        let xLow = x 0. |> int
        let xHigh = x (float highMark) |> int
        let sp1 = toScreenP h (Point(xLow,0))
        let sp2 = toScreenP h (Point(xHigh,highMark))
        let l = LineSegmentPoint(sp1,sp2)
        //printfn "%A,%A" l.P1 l.P2
        l

//Returns a new image that
//projects lane lines on the input image
//utilizing the edges image of the input image
let markLanes parms (input:Mat) (edges:Mat)=
    let output = input.Clone()
    let h = input.Height
    let segStd = Cv2.HoughLinesP(!>edges,1.,Math.PI/180.,parms.HoughTh,parms.HoughMinLineLn,parms.HoughMaxGap)
    use mHough = input.EmptyClone()
    let (sL,sR) = segmentLanes h parms segStd
    let segLeft,segRight = laneOverlay h parms sL, laneOverlay h parms sR
    //segStd |> Array.iter(fun l-> mHough.Line(l.P1,l.P2,!>130.,5)) //draw all segments found
    let color = Scalar(255.,100.,100.)
    mHough.Line(segLeft.P1,segLeft.P2,color,10)
    mHough.Line(segRight.P1,segRight.P2,color,10)
    use mTemp = new Mat()
    Cv2.CvtColor(!>edges,!>mTemp,ColorConversionCodes.GRAY2RGB)
    use cm = mTemp.InRange(Scalar(200.,200.,200.),Scalar(255.,255.,255.))
    use mTemp2 = mTemp.SetTo(Scalar(0.,0.,255.),cm)
    Cv2.AddWeighted(!>mHough,0.8,!>input,1.0,0.,!>output)
    //Cv2.AddWeighted(!>output,1.0,!>mTemp2,1.0,1.0,!>output) //draw the edge marker 
//    segStd |> Array.iteri(fun i l ->     
//        Cv2.PutText(!>output,sprintf "%d:%A" i (slope h l),l.P2,HersheyFonts.HersheyPlain,1.,!>255.,1)
//        Cv2.PutText(!>mColor2,sprintf "p1:%d,%d" l.P1.X l.P1.Y,l.P1,HersheyFonts.HersheyPlain,1.,!>255.,1)
//        Cv2.PutText(!>mColor2,sprintf "p2:%d,%d" l.P2.X l.P2.Y,l.P2,HersheyFonts.HersheyPlain,1.,!>255.,1)
//        )
    output

;;

//code to process image and video files 
//======================================
open System.IO
let inImageFiles = Directory.GetFiles(@"C:\Users\family\CarND-LaneLines-P1\test_images")
let outImageFolder = @"C:\Users\family\CarND-LaneLines-P1\test_images_output"

let inputVideo = @"C:\Users\family\CarND-LaneLines-P1\test_videos\challenge.mp4"
let outputVideo = @"C:\Users\family\CarND-LaneLines-P1\test_videos\challengeOut.mp4"

let processVideoImage parms (mask:Mat) (mImage:Mat) = markLanes parms mImage (markEdges parms mImage mask)

let processVideo parms inputFile outputFile =
    use clipIn = new VideoCapture(inputFile:string)
    let (w,h) = clipIn.FrameWidth,clipIn.FrameHeight
    use clipOut = new VideoWriter()
    clipOut.Open(outputFile,FourCC.DIVX,clipIn.Fps,Size(w,h))
    if not(clipOut.IsOpened()) then failwith "file not opened"
    let mask,maskPts = makeMask (w,h)
    let m2 = new Mat(Size(w,h),MatType.CV_8UC3)
    Cv2.FillConvexPoly(!>m2,maskPts,Scalar(255.,255.,255.))
    while clipIn.Grab() do
        let img = clipIn.RetrieveMat()
        let outImg = processVideoImage parms mask img
//        Cv2.AddWeighted(!>m2,0.2,!>outImg,1.0,0.,!>outImg) //draws the ploygon of interest view on video
        clipOut.Write(outImg)
        img.Release()
        outImg.Release()
    clipIn.Release()
    clipOut.Release()
    mask.Release()
    m2.Release()    
(*

processVideo Parms.Default inputVideo outputVideo

*)
let frameoutFolder = @"C:\Users\family\CarND-LaneLines-P1\optframes"

let extractFrames inputFile outputFolder =
    use clipIn = new VideoCapture(inputFile:string)
    let mutable frame = 0
    while clipIn.Grab() do
        let img = clipIn.RetrieveMat()
        frame <- frame + 1
        let path = Path.Combine(outputFolder,sprintf "i%d.jpg" frame)
        if not (img.SaveImage(path)) then failwithf "unable to save frame %d" frame
        img.Release()
(*
extractFrames inputVideo frameoutFolder
*)

let processImageFile parms inputFile outputFolder =
    use mImage = new Mat(inputFile, ImreadModes.AnyColor)
    let (mask,_) = makeMask (mImage.Width,mImage.Height)
    use mEdges = markEdges parms mImage mask
    use o = markLanes parms mImage mEdges
    let fn = Path.GetFileName(inputFile)
    let ofn = Path.Combine(outputFolder,fn)
    o.SaveImage(ofn) |> ignore
    mask.Release()

let findSlope parms inputFile outputFolder =
    use mImage = new Mat(inputFile, ImreadModes.AnyColor)
    let (mask,_) = makeMask (mImage.Width,mImage.Height)
    use edges = markEdges parms mImage mask
    let segStd = Cv2.HoughLinesP(!>edges,1.,Math.PI/180.,parms.HoughTh,parms.HoughMinLineLn,parms.HoughMaxGap)
    let h = mImage.Height
    let (sL,sR) = segmentLanes h parms segStd
    let segLeft,segRight = laneOverlay h parms sL, laneOverlay h parms sR
    segLeft,segRight

let processOptFolder() =
    let o = @"C:\Users\family\CarND-LaneLines-P1\optframeLabels"
    let parms = LParms.Default
    for f in Directory.GetFiles(frameoutFolder) do
        processImageFile parms f o

let processSlopes() =
    let o = @"C:\Users\family\CarND-LaneLines-P1\optframeLabels"
    let parms = LParms.Default
    let data =
        [for f in Directory.GetFiles(frameoutFolder) do
            let (l1,l2) = findSlope parms f o
            yield
                sprintf 
                    "%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d" 
                    f
                    l1.P1.X l1.P1.Y l1.P2.X l1.P2.Y
                    l2.P1.X l2.P1.Y l2.P2.X l2.P2.Y
         ]
    File.WriteAllLines(Path.Combine(o,"lables.txt"),data)
    