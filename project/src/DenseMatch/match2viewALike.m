function pair = match2viewALike(frames, frameID_i, frameID_j, datapath)
pair.frames = [frameID_i, frameID_j];

pair.matches = -load(['images' datapath '/pair' num2str(frameID_i) num2str(frameID_j) '.txt']);

end