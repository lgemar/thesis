% Auto-generated by cameraCalibrator app on 16-Mar-2016
%-------------------------------------------------------


% Define images to process
imageFileNames = {'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image1.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image2.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image3.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image4.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image5.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image6.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image7.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image8.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image9.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image10.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image11.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image12.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image13.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image14.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image15.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image16.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image17.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image18.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image19.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image20.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image21.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image22.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image23.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image24.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image25.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image26.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image27.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image28.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image29.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image30.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image31.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image32.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image33.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image34.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image35.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image36.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image37.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image38.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image39.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image40.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image41.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image42.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image43.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image44.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image45.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image46.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image47.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image48.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image49.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\Image50.png',...
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
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
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
