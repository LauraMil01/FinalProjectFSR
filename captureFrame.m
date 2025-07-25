function captureFrame(fig, videoObj)
   
    if isempty(fig) || ~isvalid(fig)
        return
    end

    frame = getframe(fig);
    
    writeVideo(videoObj, frame);
end
