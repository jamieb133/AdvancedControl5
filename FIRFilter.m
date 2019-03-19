classdef (ConstructOnLoad = true) FIRFilter  < handle
    properties 
        taps %number of filter coefficients
        coeffs %impulse response (array of coefficients)
        buffer %buffer containing previous samples
    end %properties
    methods 
        %constuctor
        function self = FIRFilter(coeffs)
            tapSize = size(coeffs)
            self.taps = tapSize(2)
            self.coeffs = coeffs
            self.buffer = ones(1, self.taps - 1)
        end
        
        %filters samples via convolution
        function outSample = filter(self, inSample)
            outSample = 0;
            %shift data along buffer by one sample
            self.buffer;
            self.taps;
            for count = self.taps:-1:2
                self.buffer(count) = self.buffer(count-1);
            end;           
            %insert new sample
            self.buffer(1) = inSample;
            %convolve
            for count = 1 : (self.taps - 1)
                outSample = outSample + self.buffer(count) * self.coeffs(count);                
            end
        end % filter()
    end %methods
end