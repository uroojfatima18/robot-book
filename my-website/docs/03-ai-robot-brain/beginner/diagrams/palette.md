# Color-Blind Safe Diagram Palette

**Reference**: NFR-002 - All diagrams must use color-blind safe palettes

---

## Primary Palette (Wong 2011)

This palette is designed to be distinguishable by people with the most common forms of color blindness (deuteranopia, protanopia, tritanopia).

| Color Name | Hex Code | RGB | Usage |
|------------|----------|-----|-------|
| Black | #000000 | (0, 0, 0) | Text, borders, primary lines |
| Orange | #E69F00 | (230, 159, 0) | Sensor data, input |
| Sky Blue | #56B4E9 | (86, 180, 233) | Processing, algorithms |
| Bluish Green | #009E73 | (0, 158, 115) | Output, success states |
| Yellow | #F0E442 | (240, 228, 66) | Warnings, highlights |
| Blue | #0072B2 | (0, 114, 178) | Navigation, planning |
| Vermillion | #D55E00 | (213, 94, 0) | Errors, obstacles |
| Reddish Purple | #CC79A7 | (204, 121, 167) | Learning, RL components |

---

## Usage Guidelines

### Perception Diagrams
- **Sensor input**: Orange (#E69F00)
- **Processing nodes**: Sky Blue (#56B4E9)
- **Perception output**: Bluish Green (#009E73)

### Navigation Diagrams
- **Global path**: Blue (#0072B2)
- **Local path**: Sky Blue (#56B4E9)
- **Obstacles**: Vermillion (#D55E00)
- **Goal**: Bluish Green (#009E73)

### SLAM Diagrams
- **Robot pose**: Orange (#E69F00)
- **Map features**: Blue (#0072B2)
- **Scan data**: Sky Blue (#56B4E9)
- **Loop closures**: Reddish Purple (#CC79A7)

### Reinforcement Learning Diagrams
- **Agent**: Orange (#E69F00)
- **Environment**: Blue (#0072B2)
- **Reward signal**: Bluish Green (#009E73)
- **Policy/Value**: Reddish Purple (#CC79A7)

---

## SVG Template

```xml
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 800 600">
  <style>
    .text { font-family: sans-serif; font-size: 14px; fill: #000000; }
    .input { fill: #E69F00; }
    .process { fill: #56B4E9; }
    .output { fill: #009E73; }
    .warning { fill: #F0E442; }
    .navigation { fill: #0072B2; }
    .obstacle { fill: #D55E00; }
    .learning { fill: #CC79A7; }
  </style>
  <!-- Diagram content here -->
</svg>
```

---

## Accessibility Requirements

1. **Never rely on color alone** - Use shapes, patterns, or labels in addition to color
2. **Sufficient contrast** - Text must have 4.5:1 contrast ratio minimum
3. **Alt-text required** - All SVG diagrams must include descriptive alt-text
4. **Pattern fills** - Consider hatching or dotting for additional differentiation

---

## Testing Tools

- [Coblis Color Blindness Simulator](https://www.color-blindness.com/coblis-color-blindness-simulator/)
- [Viz Palette](https://projects.susielu.com/viz-palette) - Check palette accessibility
- Browser extensions for simulating color blindness

---

## References

- Wong, B. (2011). "Color blindness". Nature Methods 8, 441.
- [Okabe & Ito Color Universal Design](https://jfly.uni-koeln.de/color/)
