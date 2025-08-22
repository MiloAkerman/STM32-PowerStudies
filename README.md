# Acoustic Collar - STM32 Dev and Power Studies
*Last updated 8/22/2025*

This repo is intended to provide a comprehensive code base and documentation for the acoustic collar project at E4E. The current branches, in descending order of date last updated, are as follows:
- `cm4-active`: Testing SAI. DMA move to Cortex-M4 core for dual-core wake-up system
- `LoRa`: LoRa integration for data uplink
- `RTC`: Real-Time Clock integration for data labelling
- `inference` (Obsolete): Testing inference
- `nothing` (Obsolete): Baseline for power tests

## Project Details
- Target board: STM32H747I-DISCO (future move intended to smaller board)
- Input: On-board MEMS microphone
- Output: SD card write or LoRa (in progress)
- Power estimates: See [blog post](https://docs.google.com/document/d/1pEJw7hPAgSaujo6Jzskwartr97DzW8AzQoRQzoEKCUI/edit?usp=sharing)

## System Outline
- The *SAI4/BDMA* collect audio from the onboard MEMS microphone and store it in a circular buffer
- Optionally, the *SAI1/DMA* pipe audio data to the headphones for debugging playback
- The *X-CUBE-AI* middleware on the Cortex-M7 core processes the audio data into a mel spectrogram and runs inference
- The *SDMMC* stores the inference result and audio .wav file on local SD card

## Development Tools
- STM32CubeIDE
    - Import project from menu
- STM32CubeH7 library and drivers

## Next Steps
- [ ] Complete Cortex-M4 integration for DSP
- [ ] Integrate LoRa for audio uplink
- [ ] Build and attach autoencoder for audio compression
- [ ] Test inference  