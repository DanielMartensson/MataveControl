% Generates a ARMA model from numerator vector and denomerator vector
% Input: numerator, denomerator, sampleTime(optional), delay(optional)
% Example 1: H = arma(num, den)
% Example 2: H = arma(num, den, sampleTime)
% Example 3: H = arma(num, den, sampleTime, delay)
% Author: Daniel Mårtensson, 2022 Oktober

function [H] = arma(varargin)
  % Check if there is some input arguments
  if(isempty (varargin))
    error ('Missing arguments')
  end
  % OK. We have arguments!

  % Get numerator
  if(length(varargin) < 1)
    error('Missing numerator')
  else
    numerator = varargin{1};
  end

  % Get denomerator
  if(length(varargin) < 2)
    error('Missing denomerator')
  else
    denomerator = varargin{2};
  end

  % Get sampleTime
  if(length(varargin) >= 3)
    sampleTime = varargin{3};
  else
    sampleTime = 0;
  end

  % Check if there is some numerators
  if(isempty (numerator))
    error('Missing numerator');
  end
  % OK. We have numerators!

  % Check if there is some denomerators
  if(isempty (denomerator))
    error('Missing denomerator');
  end
  % OK. We have denomerator!

  % Make sure that we divide all with denomerator(1)
  if(denomerator(1) == 0)
    error('First number in the denomerator cannot be 0');
  else
    numerator = numerator/denomerator(1);
    denomerator = denomerator/denomerator(1);
  end

  % Create transfer function
  H.num = numerator;
  H.den = denomerator;
  H.delay = 0;

  % Add numerator
  tfnum = '';
  firstNumber = false;
  for i = 1:length(numerator)
    a = numerator(i);
    if(a > 0)
      if(or(i == 1, firstNumber == false))
        tfnum = strcat(tfnum, num2str(a), sprintf('z^-%i', i-1));
        firstNumber = true;
      else
        tfnum = strcat(tfnum, {' + '}, num2str(a), sprintf('z^-%i', i-1));
      end
    elseif(a < 0)
      tfnum = strcat(tfnum, {' '}, num2str(a), sprintf('z^-%i', i-1));
    end
  end
  H.tfnum = char(tfnum);

  % Add the dash lines
  H.tfdash = '-';

  % Add denomerator
  tfden = '1';
  for i = 2:length(denomerator)
    a = denomerator(i);
    if(a > 0)
      tfden = strcat(tfden, {' + '}, num2str(a), sprintf('z^-%i', i-1));
    elseif(a < 0)
      tfden = strcat(tfden, {' '}, num2str(a), sprintf('z^-%i', i-1));
    end
  end
  H.tfden = char(tfden(1,1));

  % Update the dash line
  H.tfdash = getDashedLine(H.tfnum, H.tfden);

  % Add type and sample time
  H.type = 'ARMA';
  H.sampleTime = sampleTime;
end


function [dash] = getDashedLine(numeratorString, denomeratorString)
  dash = '';
  if(length(numeratorString) <= length(denomeratorString))
    for i = 1:length(denomeratorString)
      dash = strcat(dash,'-');
    end
  else
    for i = 1:length(numeratorString)
      dash = strcat(dash,'-');
    end
  end
end
