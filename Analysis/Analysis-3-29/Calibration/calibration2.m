% Auto-generated by cameraCalibrator app on 30-Mar-2016
%-------------------------------------------------------


% Define images to process
imageFileNames = {'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image1.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image2.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image3.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image4.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image5.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image6.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image7.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image8.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image9.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image10.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image11.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image12.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image13.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image14.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image15.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image16.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image17.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image18.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image19.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image20.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image21.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image22.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image23.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image24.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image25.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image26.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image27.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image28.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image31.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image32.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image33.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image34.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image35.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image36.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image37.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image38.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image39.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image40.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image41.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image46.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image47.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image48.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image49.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image50.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image51.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image52.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image53.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image54.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image55.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image56.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image57.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image58.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image59.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image60.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image61.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image62.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image63.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image64.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image65.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image66.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image67.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image68.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image69.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image70.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image71.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image72.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image73.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image74.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image76.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image77.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image78.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image79.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image80.png',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Generate world coordinates of the corners of the squares
squareSize = 22;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'mm', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams, 'BarGraph');

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
originalImage = imread(imageFileNames{1});
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
