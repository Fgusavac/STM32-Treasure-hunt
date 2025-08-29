#ifndef SONG_H
#define SONG_H

#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988


extern int notes[];
extern int durations[];
extern const int song_length;
extern const int song_length_third;
extern const int song_length_half;

extern int play_song_2;
extern int play_song_3;

#endif
