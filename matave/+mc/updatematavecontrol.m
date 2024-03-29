% Update the whole matavecontrol library by downloading from GitHub
% Author: Daniel Mårtensson, Februari 2018
% Update 2023-05-15 for MATLAB Control System Toolbox portability

function updatematavecontrol()
  % Get the current working dictionary
  currentFolder = pwd;

  % Just update matavecontrol
  A = what('matave'); % Importat that it must stand 'matave'. Nothing else!

  % Get the information about where the files are
  path = strcat(A.path, '\', '+mc'); % File path

  % Go to path dictionary
  cd(path);

  % Downloading listOfFunctions
  if(exist('OCTAVE_VERSION', 'builtin') ~= 0)
    urlwrite('https://raw.githubusercontent.com/DanielMartensson/matavecontrol/master/listOfFunctions', 'listOfFunctions');
  else
     websave('https://raw.githubusercontent.com/DanielMartensson/matavecontrol/master/listOfFunctions', 'listOfFunctions');
  end

  % Read the listOfFunctions
  fid = fopen('listOfFunctions');
  txt = textscan(fid, '%s', 'delimiter', '\n'); % Importat to have the delimiter = '\n'
  % Remove this double struct
  fileList = txt{1,1};

  % Downloading all the other files
  [m, n] = size(fileList);
  for i = 1:m
    % Downloading
    nameOfFile = fileList{i, 1};
    URL = strcat('https://raw.githubusercontent.com/DanielMartensson/matavecontrol/master/matave/+mc/', nameOfFile)
    [saveplace, Success] = urlwrite(URL, nameOfFile);
    saveplace
    Success
  end

  disp('Matavecontrol is updated!')

  % Get to the current working dictionary
  cd(currentFolder);

end
