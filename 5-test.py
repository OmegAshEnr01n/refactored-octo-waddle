import imageio

w = imageio.get_writer('_out/my_video.mp4', format='FFMPEG', mode='I', fps=1)


i = imageio.imread('img.jpeg')

w.append_data(i)

w.close

