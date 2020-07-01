function mapZ = loadMapFromImage(imgFilename)

% load image (m,n,3) pixels - [3 for each R,G,B]
imagem  = imread(imgFilename);

% sum through RGB dimension to find all non-zero (i.e 'not white') pixels,
%   where white indicates unoccupied space.
imgsum      = sum(imagem,3);
mapZ        = zeros(size(imgsum));

% set all 'non-white' pixels to 1 (i.e occupied)
mapZ(imgsum~=0) = 0;
mapZ(imgsum==0) = 1;

% reverse north direction for some reason
mapZ = mapZ(end:-1:1,:);
