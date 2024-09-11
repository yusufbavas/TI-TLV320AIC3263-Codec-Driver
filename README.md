Add tlv320AIC3263 codec chip support to driver given in ti forum (https://e2e.ti.com/support/audio-group/audio/f/audio-forum/1104527/tlv320aic3263-linux-kernel-v5-4-driver/4097927)

Fix mode selection issue in driver. https://e2e.ti.com/support/audio-group/audio/f/audio-forum/1131446/tlv320aic3263-aic3263-mode-selection/4211112)

### Simple device tree bindings example:

    &i2c0 {
        tlv320aic326x: tlv320aic326x@18 {
            compatible = "ti,aic3262";
            reg = <0x18>;
            pinctrl-names = "default";
            pinctrl-0 = <&pinctrl_aic3263>;
            #sound-dai-cells = <0>;
            status = "okay";
        };
    };

    sound-tlv320aic326x {
        compatible = "simple-audio-card";
        simple-audio-card,name = "tlv320aic3262-hifi";
        simple-audio-card,format = "i2s";
        simple-audio-card,bitclock-master = <&codec_cpu>;
        simple-audio-card,frame-master = <&codec_cpu>;

        codec_cpu: simple-audio-card,cpu {
            sound-dai = <&sai0>;
        };

        codec_dai: simple-audio-card,codec {
            sound-dai = <&tlv320aic326x>;
        };
    };
