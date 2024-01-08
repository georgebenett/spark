# Spark

This is code for a RIGADO BMD340 module   

The code can be build with the NRF52 SDK by changing the path in Makefile. 

The first time the code is uploaded mass_erase must be run, and the softdevice must be uploaded. This can be done with the following make rules:

```bash
make mass_erase
```

```bash
make upload_sd
```

after that and after future changes only the code itself needs to be uploaded:

```bash
make upload
```
