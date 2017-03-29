#I @"..\packages\OpenCvSharp3-AnyCPU.3.2.0.20170324\lib\net40"
#r @"OpenCvSharp.dll"
#r @"OpenCvSharp.Blob.dll"
#r @"OpenCvSharp.Extensions.dll"
#r @"OpenCvSharp.UserInterface.dll"
System.Environment.CurrentDirectory <- @"C:\Users\family\Documents\Visual Studio 2015\Projects\OpenCVSharpTest\packages\OpenCvSharp3-AnyCPU.3.2.0.20170324\NativeDlls\x64"
let uiCtx = System.Threading.SynchronizationContext.Current
open OpenCvSharp
open System

let win t i = 
    async{
        do! Async.SwitchToContext uiCtx
        new Window((t:string), WindowMode.AutoSize,i) |> ignore 
        } |> Async.Start

let inline (!>) (x:^a) : ^b = ((^a or ^b) : (static member op_Implicit : ^a -> ^b) x)

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

let isolateLaneMarkers(mImage:Mat)=
    let m = new Mat()
    let low = Scalar(90.,90.,190.)
    let high = Scalar(255.,255.,255.)
    Cv2.InRange(!>mImage,low,high,!>m)
    m

let markEdges (mImage:Mat) (mask:Mat) =
    let gray = isolateLaneMarkers mImage
    Cv2.BitwiseAnd(!>gray,!>mask,!>gray)
    Cv2.Canny(!>gray,!>gray,100.,200.,3)
    gray

let leftLaneSlopes = (0.45,0.72)
let rightLaneSlopes = (-0.65,-0.45)
let (<->) s (a,b)  = s >= a && s <= b 

let segmentLanes h (segStds:LineSegmentPoint[]) =
    (([],[]),segStds) ||> Array.fold(fun (ls,rs) seg ->
        let s = slope h seg
        if s <-> rightLaneSlopes then 
            (ls,(s,seg)::rs)
        elif s <-> leftLaneSlopes then
            ((s,seg)::ls,rs)
        else
            (ls,rs)
        )

let NullSeg = LineSegmentPoint(Point(0,0),Point(0,0))

let laneOverlay h (segs:(float*LineSegmentPoint) seq) =
    if segs |> Seq.isEmpty then 
        NullSeg
    else
        let avgTheta = segs |> Seq.averageBy fst
        let pxI = segs |> Seq.collect (fun (_,s) -> [s.P1.X;s.P2.X]) |> Seq.map float |> Seq.average |> int
        let pyI = segs |> Seq.collect (fun (_,s) -> [s.P1.Y;s.P2.Y]) |> Seq.map float |> Seq.average |> int
        //        let (_,fseg) = segs |> Seq.maxBy (fun (_,s)->s.P1.Y)
        let pW = toWorldP h (Point(pxI,pyI)) //fseg.P2
        let (px,py) = float pW.X, float pW.Y
        let slope = Math.Tan(avgTheta) 
        //(y - y1) = m(x - x1) ==> (y - y1) = Tan(theta) (x - x1) ==> x = (y - y1)/(Tan(theta)) + x1
        let y x' = (slope * (x' - px) ) - py
        let x y' = ((y' - py)/slope) + px
        let highMark = 200
        let xLow = x 0. |> int
        let xHigh = x (float highMark) |> int
        let sp1 = toScreenP h (Point(xLow,0))
        let sp2 = toScreenP h (Point(xHigh,highMark))
        let l = LineSegmentPoint(sp1,sp2)
        //printfn "%A,%A" l.P1 l.P2
        l

let markLanes (input:Mat) (edges:Mat)=
    let output = input.Clone()
    let h = input.Height
    let segStd = Cv2.HoughLinesP(!>edges,1.,Math.PI/180.,15,30.,100.)
    use mHough = input.EmptyClone()
    let (sL,sR) = segmentLanes h segStd
    let segLeft,segRight = laneOverlay h sL, laneOverlay h sR
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

//edges(mImage) |> win "i"
;;
open System.IO
let inImageFiles = Directory.GetFiles(@"C:\Users\family\CarND-LaneLines-P1\test_images")
let outImageFolder = @"C:\Users\family\CarND-LaneLines-P1\test_images_output"

let processFileImage f =
    use mImage = new Mat(f, ImreadModes.AnyColor)
    let (mask,_) = makeMask (mImage.Width,mImage.Height)
    use mEdges = markEdges mImage mask
    use o = markLanes mImage mEdges
    let fn = Path.GetFileName(f)
    let ofn = Path.Combine(outImageFolder,fn)
    o.SaveImage(ofn) |> ignore
    mask.Release()

let videoInFolder = @"C:\Users\family\CarND-LaneLines-P1\test_videos"
let videoInFiles = Directory.GetFiles videoInFolder
let videoOutFolder = @"C:\Users\family\CarND-LaneLines-P1\test_videos_output"

let processVideoImage (mask:Mat) (mImage:Mat) = markLanes mImage (markEdges mImage mask)

let processVideo f =
    printfn "processing: %s" f
    let fn = Path.GetFileName(f)
    let ofn = Path.Combine(videoOutFolder,fn)
    printfn "%s" ofn
    use clipIn = new VideoCapture(f)
    let (w,h) = clipIn.FrameWidth,clipIn.FrameHeight
    use clipOut = new VideoWriter()
    clipOut.Open(ofn,FourCC.DIVX,clipIn.Fps,Size(w,h))
    if not(clipOut.IsOpened()) then failwith "file not opened"
    let mask,maskPts = makeMask (w,h)
    let m2 = new Mat(Size(w,h),MatType.CV_8UC3)
    Cv2.FillConvexPoly(!>m2,maskPts,Scalar(255.,255.,255.))
    while clipIn.Grab() do
        let img = clipIn.RetrieveMat()
        let outImg = processVideoImage mask img
//        Cv2.AddWeighted(!>m2,0.2,!>outImg,1.0,0.,!>outImg) //draws the ploygon of interest view on video
        clipOut.Write(outImg)
        img.Release()
        outImg.Release()
    clipIn.Release()
    clipOut.Release()
    mask.Release()
    m2.Release()

(*
inImageFiles |> Seq.head |> processFileImage
for f in inImageFiles do processFileImage f

let f = videoInFiles |> Seq.head
for f in videoInFiles do processVideo f
*)