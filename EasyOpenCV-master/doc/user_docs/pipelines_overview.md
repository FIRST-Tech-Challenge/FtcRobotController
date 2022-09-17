# Creating Pipelines

### What is a pipeline?

A pipeline is essentially an encapsulation of OpenCV image processing to do a certain thing. Most of the time, image processing requires operations to be done in series instead of in parallel; outputs from step A are fed into the inputs of step B, and outputs of step B are fed into step C, and so on; hence, the term "Pipeline."

### Creating a pipeline

Pipelines can be as simple or as complex as you need them to be. All pipelines are required to extend `OpenCvPipeline` and implement the `public Mat processFrame(Mat input)` method. Let's take a look at an "empty" pipeline that doesn't actually do anything with the input image:
```java
class EmptyPipeline extends OpenCvPipeline
{
    @Override
    public Mat processFrame(Mat input)
    {
        return input;
    }
}
```
The `Mat` which is returned from the `processFrame(...)` function is what will be displayed on the live preview. Above, you can see that the raw `input` image is being returned.

Let's take a look at an example that will convert the input image to black & white and display the black & white image on the screen.

> This is a good time to mention that ideally you should create any auxilliary `Mat` objects your pipeline will need as instance variables of your pipeline class instead of as local variables in the `processFrame(...)` function. By doing so, you eliminate the possibility of forgetting to call `.release()` on `Mat`s and thus iteratively leaking memory on each frame (which if allowed to go unchecked will cause an OOME and crash the Robot Controller app).

```java
class ConvertToGreyPipeline extends OpenCvPipeline
{
    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat grey = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        return grey;
    }
}
```

What about for auxilliarly `Mat` objects that need to be initialized once with a real frame, such as submats? For this case you can implement the `void init(Mat firstFrame)` function in your pipeline. This function will be called once (and only once) before the iterative calls to `processFrame(...)`

```java
class SubmatPipeline extends OpenCvPipeline
{
    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat submat;

    @Override
    public void init(Mat firstFrame)
    {
        submat = firstFrame.submat(0,50,0,50);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Because a submat is a persistent reference to a region of the parent buffer,
        // (which in this case is `input`) any changes to `input` will be reflected in
        // the submat (and vice versa).
        return submat;
    }
}
```

### How exactly are pipelines run?

As a user, you **NEVER** need to manually call the `processFrame(...)` function. If you attach the pipeline to the camera with `camera.setPipeline(pipeline)`, the camera's frame worker thread will automatically dispatch new frames to your pipeline as they become available. Note that your `processFrame(...)` function will NEVER be called concurrently. If a new frames comes in while your pipeline is still processing a preview frame, the new frame will either be dropped or queued.

Since pipelines are not run on your OpMode thread, your vision processing will not slow down your hardware control loops.

### Getting data to the OpMode thread

A pipeline with cool image processing is great and all, but won't do you much good if you can't actually obtain results from it in your robot control code. The recommended way to do this is to have your `processFrame(...)` function store the analysis results in an instance variable of the pipeline class right before it returns.

```java
class FoobarPipeline extends OpenCvPipeline
{
    int lastResult = 0;

    @Override
    public Mat processFrame(Mat input)
    {
        // ... some image processing here ...

        if(...)
        {
            lastResult = 1;
        }
        else if(...)
        {
            lastResult = 2
        }
        else if(...)
        {
            lastResult = 3;
        }
    }

    public int getLatestResults()
    {
        return lastResult;
    }
}
```

Then you can have some sort of `getLastResults()` function which can return that instance variable. From your robot control code you could then call:
```
<OBJECT_TYPE_HERE> results = pipeline.getLatestResults()
```
Since this would simply be returning the cached results from the last analysis, the robot control code would not incur any performance penalty by calling this.

### Further Information

EasyOpenCV doesn't provide any image processing functions of its own. It's simply a framework for integrating OpenCV code into the FTC SDK. Please consult the [OpenCV documentation](https://docs.opencv.org/master/) for information about the OpenCV image processing functions.