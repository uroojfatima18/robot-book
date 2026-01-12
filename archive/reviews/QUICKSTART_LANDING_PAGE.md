# Quick Start Guide - Cyber-Themed Landing Page

## Running the Landing Page

### Development Mode
```bash
cd my-website
npm run start
```
Visit `http://localhost:3000` to view the landing page.

### Production Build
```bash
npm run build
```

---

## Quick Customizations

### 1. Change Hero Title
**File**: `src/pages/index.tsx` (lines 21-25)

```typescript
<h1 className={styles.heroTitle}>
  <span className={styles.titleLine}>YOUR TEXT</span>
  <span className={styles.titleLine}>MORE TEXT</span>
  <span className={styles.titleLine}>FINAL TEXT</span>
</h1>
```

### 2. Update Stats
**File**: `src/pages/index.tsx` (lines 33-46)

```typescript
<div className={styles.statsRow}>
  <div className={styles.stat}>
    <span className={styles.statNumber}>YOUR NUMBER</span>
    <span className={styles.statLabel}>YOUR LABEL</span>
  </div>
  {/* Add more stats */}
</div>
```

### 3. Add/Remove Core Systems Cards
**File**: `src/pages/index.tsx` (lines 88-117)

```typescript
const systems = [
  {
    id: 1,
    title: 'NEW_SYSTEM',
    description: 'System description',
    status: 'ACTIVE',
    icon: '🔧', // Change emoji
  },
  // Add or remove items
];
```

### 4. Update Learning Path Chapters
**File**: `src/pages/index.tsx` (lines 155-180)

```typescript
const chapters = [
  {
    number: '01',
    difficulty: 'BEGINNER',
    title: 'Your Chapter Title',
    description: 'Your chapter description',
  },
  // Add or remove chapters
];
```

### 5. Change Code Example
**File**: `src/pages/index.tsx` (lines 204-214)

```typescript
const codeExample = `your code here`;
```

### 6. Modify Colors
**File**: `src/pages/index.module.css`

Key colors to change:
- `#00ffff` - Primary cyan
- `#00ff00` - Secondary green
- `#000000` - Black background
- `#0a0a0a` - Dark gray

Search and replace these values throughout the file.

---

## Component Reference

### Available Sections

1. **HeroSection** - Main landing area with CTA buttons
2. **CoreSystemsSection** - 4-card feature grid
3. **InteractiveModelSection** - 3D model placeholder
4. **LearningPathSection** - Chapter cards
5. **CodeExampleSection** - Code showcase with copy button
6. **SensorMapSection** - Network visualization
7. **SystemMetricsSection** - Progress bars

### Removing/Adding Sections

To **remove** a section from the page, edit `src/pages/index.tsx` line 368-374:

```typescript
export default function Home(): ReactNode {
  return (
    <Layout ...>
      <main className={styles.main}>
        <HeroSection />
        {/* Remove sections by commenting out */}
        {/* <CoreSystemsSection /> */}
        <InteractiveModelSection />
        {/* ... etc */}
      </main>
    </Layout>
  );
}
```

To **add** a new section, create a new function component and add it to the main export.

---

## Animation Control

### Slow Down Animations
**File**: `src/pages/index.module.css`

Find the animation duration and increase it:

```css
/* Before */
animation: fadeInUp 0.8s ease-out;

/* After (slower) */
animation: fadeInUp 1.2s ease-out;
```

### Disable Animations
Replace animation properties with:
```css
animation: none;
```

### Adjust Animation Delays
```css
/* Hero elements fade in with staggered delays */
animation-delay: 0.1s; /* Change this value */
```

---

## Styling Tips

### Change Card Appearance
**File**: `src/pages/index.module.css` (search for `.systemCard`)

```css
.systemCard {
  padding: 2rem;              /* Internal spacing */
  border: 2px solid #00ffff;  /* Border style */
  border-radius: 8px;         /* Corner roundness */
  background: linear-gradient(...); /* Background */
}
```

### Modify Button Styles
**File**: `src/pages/index.module.css` (search for `.buttonPrimary`)

```css
.buttonPrimary {
  padding: 1rem 2.5rem;       /* Button size */
  background: linear-gradient(...); /* Colors */
  box-shadow: 0 0 20px ...;   /* Glow effect */
}
```

### Adjust Responsive Breakpoints
**File**: `src/pages/index.module.css` (bottom of file)

```css
/* Change 768px to your preferred tablet breakpoint */
@media screen and (max-width: 768px) {
  /* Mobile styles */
}
```

---

## Common Issues & Solutions

### Animations Not Playing
**Check**: CSS module imports in `index.tsx`
```typescript
import styles from './index.module.css';
```

### Styling Not Applied
1. Clear browser cache (Ctrl+Shift+Delete)
2. Hard refresh (Ctrl+F5)
3. Run `npm run build` and check for errors

### Buttons Not Linking
**Check**: Link component in `index.tsx`
```typescript
<Link to="/docs/intro" className={styles.buttonPrimary}>
  BUTTON TEXT
</Link>
```
Ensure the path exists in your Docusaurus config.

### Mobile Layout Broken
1. Check viewport meta tag in Docusaurus HTML head
2. Verify responsive media queries in CSS
3. Test in browser dev tools (F12)

---

## Performance Optimization

### Reducing Bundle Size
- Remove unused animations
- Compress background gradients
- Minimize color variables

### Improving Load Time
1. Use CSS modules (already implemented)
2. Lazy load sections with React.lazy (optional)
3. Preload fonts (if using custom fonts)

### Best Practices
- Keep animations under 1s for better performance
- Use `will-change` for frequently animated elements
- Limit the number of active animations simultaneously

---

## Browser Compatibility

| Feature | Chrome | Firefox | Safari | Edge |
|---------|--------|---------|--------|------|
| CSS Grid | ✓ | ✓ | ✓ | ✓ |
| Flexbox | ✓ | ✓ | ✓ | ✓ |
| CSS Animations | ✓ | ✓ | ✓ | ✓ |
| SVG | ✓ | ✓ | ✓ | ✓ |
| Gradient Text | ✓ | ✓ | ✓ | ✓ |

---

## Testing Checklist

Before deploying, verify:
- [ ] Page loads without console errors
- [ ] All sections display correctly
- [ ] Buttons navigate to correct URLs
- [ ] Responsive design works (test mobile view)
- [ ] Animations are smooth
- [ ] Copy button works in code section
- [ ] Hover effects display properly
- [ ] Text is readable on all screen sizes

---

## Deployment Checklist

Before production deployment:
1. Run `npm run build` and verify success
2. Test production build locally: `npm run serve`
3. Check all links work correctly
4. Verify mobile responsiveness
5. Test on multiple browsers
6. Check accessibility (use axe DevTools)
7. Optimize images (if any added)
8. Test on slow connections (DevTools throttling)

---

## Support & Resources

### Files to Reference
- `LANDING_PAGE_IMPLEMENTATION.md` - Full implementation details
- `LANDING_PAGE_VISUAL_GUIDE.md` - Design specifications
- `src/pages/index.tsx` - Component source code
- `src/pages/index.module.css` - All styling
- `src/css/custom.css` - Global theme

### Docusaurus Documentation
- https://docusaurus.io/docs - Official docs
- https://docusaurus.io/docs/api - API reference
- https://docusaurus.io/docs/styling-layout - Styling guide

### CSS Resources
- MDN Web Docs (CSS animations, grid, flexbox)
- Can I Use (browser compatibility)
- ColorHunt.co (color palette inspiration)

---

## Quick Command Reference

```bash
# Development
npm run start              # Start dev server
npm run build             # Build for production
npm run serve             # Serve production build

# Code Quality
npm run lint              # Lint TypeScript
npm run format            # Format code
npm run test              # Run tests (if configured)

# Cleanup
rm -rf build/             # Remove build directory
npm install               # Reinstall dependencies
```

---

## Next Steps

1. Review the implementation files
2. Customize content to match your book
3. Update links to point to correct documentation
4. Add your own images/assets as needed
5. Test thoroughly on all devices
6. Deploy with confidence!

---

## Questions or Issues?

Refer to the detailed documentation files:
- Implementation details → `LANDING_PAGE_IMPLEMENTATION.md`
- Visual specifications → `LANDING_PAGE_VISUAL_GUIDE.md`
- Source code → `src/pages/index.tsx` and `index.module.css`

---

Happy customizing! The landing page is designed to be easily modified while maintaining the cyber-aesthetic. Feel free to experiment with colors, animations, and layouts.
