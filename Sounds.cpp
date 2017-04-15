#include "Sounds.hpp"

#include <sound_play/sound_play.h>
#include "boost/thread/thread.hpp"

void playSound(const std::string& filepath)
{
    sound_play::SoundClient sc;

    sc.playWave(filepath);

    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
}
