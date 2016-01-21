function tracker = initializeTracker(rgb_frame)

hsv_image = rgb2hsv(rgb_frame); 
shapeInserter = vision.ShapeInserter('BorderColor','Custom','CustomBorderColor',[1 0 0]);
tracker = vision.HistogramBasedTracker;
picfig = figure; imshow(rgb_frame); title('Bounding Box');
objectRegion=round(getPosition(imrect));
initializeObject(tracker, hsv_image(:, :, 1), objectRegion)

close(picfig)
