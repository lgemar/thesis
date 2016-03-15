function out = VectorNorm( Mat )
% Treats rows or columns along the smaller dimension as vectors and returns
% a vector of norms of all those vectors. 

if( size(Mat,1) > size(Mat,2) )
    out = sqrt(sum(Mat.^2,2)); 
else
    out = sqrt(sum(Mat.^2,1)); 
end

end