# zmk-input-processor-axis-constrain

ZMK input processor that constrains trackball movement to a single axis (horizontal or vertical).  
Useful for text selection and precise cursor control.

## Installation

Add to `config/west.yml`:

```yaml
manifest:
  remotes:
    - name: matchey
      url-base: https://github.com/matchey
  projects:
    - name: zmk-input-processor-axis-constrain
      remote: matchey
      revision: main
```

Enable in `.conf`:

```conf
CONFIG_ZMK_INPUT_PROCESSOR_AXIS_CONSTRAIN=y
```

## Usage

```
#include <axis-constrain.dtsi>
```

```dts
/ {
    zip_axis_constrain: zip_axis_constrain {
        threshold = <10>;
        sticky;
        release-after-ms = <150>;
    };
};

&trackball_listener {
    cardinal_snipe {
        layers = <4>;
        input-processors = <&zip_axis_constrain>, <&zip_xy_scaler 1 2>;
    };
};
```

## Properties

| Property | Default | Description |
|----------|---------|-------------|
| `threshold` | 5 | Movement threshold before axis is determined |
| `sticky` | false | Lock axis until movement stops |
| `release-after-ms` | 100 | Timeout to release axis lock (when sticky) |

## License

BSD 3-Clause License. See [LICENSE](LICENSE) for details.
