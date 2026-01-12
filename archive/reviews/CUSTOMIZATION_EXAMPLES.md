# Customization Examples

This document provides practical examples of how to customize the landing page.

---

## Example 1: Change Color Scheme to Purple & Blue

### Step 1: Update CSS Variables
**File**: `src/pages/index.module.css`

Replace all instances of:
- `#00ffff` (cyan) → `#9d4edd` (purple)
- `#00ff00` (green) → `#5a189a` (deep purple)

**Quick Find & Replace:**
```css
/* Search and replace in the file */
#00ffff → #9d4edd
#00ff00 → #5a189a
```

### Step 2: Update Global Theme
**File**: `src/css/custom.css`

```css
:root {
  --ifm-color-primary: #9d4edd;
  --ifm-color-primary-dark: #7b2cbf;
  --ifm-color-primary-darker: #5a189a;
  --ifm-color-primary-darkest: #3c096c;
  --ifm-color-primary-light: #b597f6;
  --ifm-color-primary-lighter: #c8b6ff;
  --ifm-color-primary-lightest: #e0aaff;
}
```

**Result**: Entire site switches to purple theme while maintaining structure.

---

## Example 2: Add a New Feature Card

### Step 1: Update Component
**File**: `src/pages/index.tsx` (lines 88-117)

```typescript
function CoreSystemsSection() {
  const systems = [
    {
      id: 1,
      title: 'NEURAL_CORE',
      description: 'Transformer Architecture',
      status: 'ACTIVE',
      icon: '🧠',
    },
    {
      id: 2,
      title: 'PERCEPTION',
      description: 'Multi-Modal Fusion',
      status: 'ONLINE',
      icon: '👁️',
    },
    {
      id: 3,
      title: 'RESPONSE_TIME',
      description: '< 10ms Latency',
      status: 'OPTIMAL',
      icon: '⚡',
    },
    {
      id: 4,
      title: 'ACTUATORS',
      description: '44 Degrees of Freedom',
      status: 'CALIBRATED',
      icon: '🎯',
    },
    // NEW CARD ADDED
    {
      id: 5,
      title: 'ENERGY_MGMT',
      description: 'Optimized Battery Life',
      status: 'EFFICIENT',
      icon: '⚙️',
    },
  ];

  return (
    <section className={styles.coreSystemsSection}>
      <div className={styles.sectionLabel}>CORE_SYSTEMS</div>
      <div className={styles.cardGrid}>
        {systems.map((system) => (
          <div key={system.id} className={styles.systemCard}>
            <div className={styles.cardIcon}>{system.icon}</div>
            <h3 className={styles.cardTitle}>{system.title}</h3>
            <p className={styles.cardDescription}>{system.description}</p>
            <div className={styles.cardStatus}>
              <span className={styles.statusBadge}>{system.status}</span>
            </div>
          </div>
        ))}
      </div>
    </section>
  );
}
```

**Result**: New card appears in the grid automatically.

---

## Example 3: Create a New Section

### Step 1: Add Component
**File**: `src/pages/index.tsx`

Add this new component:

```typescript
function TestimonialsSection() {
  const testimonials = [
    {
      id: 1,
      quote: 'The most comprehensive robotics guide available.',
      author: 'Dr. Jane Smith',
      role: 'AI Researcher',
    },
    {
      id: 2,
      quote: 'Practical examples that actually work.',
      author: 'Prof. John Doe',
      role: 'Robotics Professor',
    },
  ];

  return (
    <section className={styles.testimonialsSection}>
      <div className={styles.sectionLabel}>USER_FEEDBACK</div>
      <h2 className={styles.sectionTitle}>WHAT EXPERTS SAY</h2>
      <div className={styles.testimonialGrid}>
        {testimonials.map((testimonial) => (
          <div key={testimonial.id} className={styles.testimonialCard}>
            <p className={styles.testimonialQuote}>"{testimonial.quote}"</p>
            <div className={styles.testimonialAuthor}>
              <p className={styles.authorName}>{testimonial.author}</p>
              <p className={styles.authorRole}>{testimonial.role}</p>
            </div>
          </div>
        ))}
      </div>
    </section>
  );
}
```

### Step 2: Add CSS
**File**: `src/pages/index.module.css` (append to file)

```css
.testimonialsSection {
  padding: 6rem 2rem;
  background: linear-gradient(180deg, rgba(0, 0, 0, 0.9) 0%, rgba(0, 10, 20, 0.8) 100%);
  border-bottom: 1px solid rgba(0, 255, 255, 0.2);
}

.testimonialGrid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  max-width: 1400px;
  margin: 0 auto;
}

.testimonialCard {
  padding: 2rem;
  background: linear-gradient(135deg, rgba(0, 255, 255, 0.05) 0%, rgba(0, 20, 40, 0.2) 100%);
  border: 2px solid #00ffff;
  border-radius: 8px;
  transition: all 0.3s ease;
}

.testimonialCard:hover {
  border-color: #00ff00;
  box-shadow: 0 0 30px rgba(0, 255, 0, 0.3);
  transform: translateY(-8px);
}

.testimonialQuote {
  font-size: 1.1rem;
  color: #aaaaaa;
  font-style: italic;
  line-height: 1.6;
  margin: 0 0 1rem 0;
}

.testimonialAuthor {
  text-align: center;
}

.authorName {
  color: #ffffff;
  font-weight: bold;
  margin: 0 0 0.25rem 0;
}

.authorRole {
  color: #00ffff;
  font-size: 0.9rem;
  margin: 0;
}

@media screen and (max-width: 768px) {
  .testimonialsSection {
    padding: 3rem 1rem;
  }

  .testimonialGrid {
    grid-template-columns: 1fr;
  }
}
```

### Step 3: Add to Main Component
**File**: `src/pages/index.tsx` (lines 360-376)

```typescript
export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout ...>
      <main className={styles.main}>
        <HeroSection />
        <CoreSystemsSection />
        <InteractiveModelSection />
        <LearningPathSection />
        <CodeExampleSection />
        <TestimonialsSection />  {/* NEW SECTION */}
        <SensorMapSection />
        <SystemMetricsSection />
      </main>
    </Layout>
  );
}
```

**Result**: New testimonials section appears between code and sensor sections.

---

## Example 4: Change Hero Title and Stats

### Before:
```typescript
<h1 className={styles.heroTitle}>
  <span className={styles.titleLine}>PHYSICAL</span>
  <span className={styles.titleLine}>AI &</span>
  <span className={styles.titleLine}>HUMANOID ROBOTS</span>
</h1>

<div className={styles.statsRow}>
  <div className={styles.stat}>
    <span className={styles.statNumber}>24</span>
    <span className={styles.statLabel}>CHAPTERS</span>
  </div>
  <div className={styles.stat}>
    <span className={styles.statNumber}>180+</span>
    <span className={styles.statLabel}>DIAGRAMS</span>
  </div>
  <div className={styles.stat}>
    <span className={styles.statNumber}>500</span>
    <span className={styles.statLabel}>CODE EXAMPLES</span>
  </div>
</div>
```

### After:
```typescript
<h1 className={styles.heroTitle}>
  <span className={styles.titleLine}>ADVANCED</span>
  <span className={styles.titleLine}>MACHINE</span>
  <span className={styles.titleLine}>LEARNING SYSTEMS</span>
</h1>

<div className={styles.statsRow}>
  <div className={styles.stat}>
    <span className={styles.statNumber}>18</span>
    <span className={styles.statLabel}>MODULES</span>
  </div>
  <div className={styles.stat}>
    <span className={styles.statNumber}>250+</span>
    <span className={styles.statLabel}>ALGORITHMS</span>
  </div>
  <div className={styles.stat}>
    <span className={styles.statNumber}>1000+</span>
    <span className={styles.statLabel}>EXAMPLES</span>
  </div>
</div>
```

---

## Example 5: Add a Subscription Form

### Step 1: Add Component
**File**: `src/pages/index.tsx`

```typescript
function NewsletterSection() {
  const [email, setEmail] = useState('');
  const [submitted, setSubmitted] = useState(false);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    // Add your newsletter API call here
    console.log('Email submitted:', email);
    setSubmitted(true);
    setTimeout(() => setSubmitted(false), 3000);
  };

  return (
    <section className={styles.newsletterSection}>
      <div className={styles.newsletterContainer}>
        <div className={styles.sectionLabel}>STAY_CONNECTED</div>
        <h2 className={styles.sectionTitle}>GET THE LATEST UPDATES</h2>
        <p className={styles.newsletterSubtitle}>
          Subscribe to receive chapters, updates, and exclusive content.
        </p>
        <form className={styles.newsletterForm} onSubmit={handleSubmit}>
          <input
            type="email"
            placeholder="your@email.com"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            className={styles.emailInput}
            required
          />
          <button type="submit" className={styles.subscribeButton}>
            SUBSCRIBE
          </button>
        </form>
        {submitted && (
          <p className={styles.successMessage}>
            Thanks for subscribing!
          </p>
        )}
      </div>
    </section>
  );
}
```

### Step 2: Add CSS
**File**: `src/pages/index.module.css`

```css
.newsletterSection {
  padding: 6rem 2rem;
  background: linear-gradient(180deg, rgba(0, 10, 20, 0.8) 0%, rgba(0, 0, 0, 0.9) 100%);
  border-bottom: 1px solid rgba(0, 255, 255, 0.2);
}

.newsletterContainer {
  max-width: 600px;
  margin: 0 auto;
  text-align: center;
}

.newsletterSubtitle {
  font-size: 1.1rem;
  color: #aaaaaa;
  margin: 1rem 0 2rem 0;
}

.newsletterForm {
  display: flex;
  gap: 1rem;
  margin-bottom: 1rem;
}

.emailInput {
  flex: 1;
  padding: 0.8rem 1rem;
  background: rgba(0, 255, 255, 0.05);
  border: 2px solid #00ffff;
  color: #00ffff;
  border-radius: 4px;
  font-family: 'Courier New', monospace;
  transition: all 0.3s ease;
}

.emailInput:focus {
  outline: none;
  background: rgba(0, 255, 255, 0.15);
  box-shadow: 0 0 20px rgba(0, 255, 255, 0.3);
}

.emailInput::placeholder {
  color: rgba(170, 170, 170, 0.7);
}

.subscribeButton {
  padding: 0.8rem 2rem;
  background: linear-gradient(90deg, #00ffff 0%, #00ff00 100%);
  color: #000;
  border: none;
  border-radius: 4px;
  font-weight: bold;
  cursor: pointer;
  transition: all 0.3s ease;
  text-transform: uppercase;
}

.subscribeButton:hover {
  box-shadow: 0 0 20px rgba(0, 255, 255, 0.6);
  transform: translateY(-2px);
}

.successMessage {
  color: #00ff00;
  text-shadow: 0 0 10px rgba(0, 255, 0, 0.5);
  animation: fadeInUp 0.5s ease-out;
}
```

---

## Example 6: Add Dark/Light Theme Toggle

### Step 1: Create Theme Hook
**File**: `src/hooks/useTheme.ts` (new file)

```typescript
import { useEffect, useState } from 'react';

export function useTheme() {
  const [isDark, setIsDark] = useState(true);

  useEffect(() => {
    const html = document.documentElement;
    if (isDark) {
      html.setAttribute('data-theme', 'dark');
    } else {
      html.removeAttribute('data-theme');
    }
  }, [isDark]);

  return { isDark, setIsDark };
}
```

### Step 2: Update Hero Section
**File**: `src/pages/index.tsx`

```typescript
function HeroSection() {
  const { isDark, setIsDark } = useTheme();

  return (
    <section className={styles.hero}>
      <button
        className={styles.themeToggle}
        onClick={() => setIsDark(!isDark)}
      >
        {isDark ? '☀️' : '🌙'}
      </button>
      {/* Rest of hero content */}
    </section>
  );
}
```

### Step 3: Add CSS
**File**: `src/pages/index.module.css`

```css
.themeToggle {
  position: fixed;
  top: 2rem;
  right: 2rem;
  padding: 0.5rem 1rem;
  background: rgba(0, 255, 255, 0.1);
  border: 2px solid #00ffff;
  color: #00ffff;
  border-radius: 50%;
  cursor: pointer;
  z-index: 100;
  transition: all 0.3s ease;
}

.themeToggle:hover {
  background: rgba(0, 255, 255, 0.2);
  box-shadow: 0 0 20px rgba(0, 255, 255, 0.4);
}
```

---

## Example 7: Modify Button Links

### Change Navigation Destinations
**File**: `src/pages/index.tsx`

```typescript
// Before
<Link to="/docs/intro" className={styles.buttonPrimary}>
  BEGIN LEARNING
</Link>

// After (link to custom page)
<Link to="/docs/getting-started" className={styles.buttonPrimary}>
  BEGIN LEARNING
</Link>

// Or external link
<a href="https://example.com" className={styles.buttonPrimary}>
  BEGIN LEARNING
</a>
```

---

## Example 8: Add Animations on Scroll

### Step 1: Create Intersection Observer Hook
**File**: `src/hooks/useInViewAnimation.ts` (new file)

```typescript
import { useEffect, useRef, useState } from 'react';

export function useInViewAnimation() {
  const ref = useRef<HTMLDivElement>(null);
  const [isInView, setIsInView] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsInView(true);
        }
      },
      { threshold: 0.1 }
    );

    if (ref.current) {
      observer.observe(ref.current);
    }

    return () => observer.disconnect();
  }, []);

  return { ref, isInView };
}
```

### Step 2: Use in Component
**File**: `src/pages/index.tsx`

```typescript
function CoreSystemsSection() {
  const { ref, isInView } = useInViewAnimation();

  return (
    <section ref={ref} className={isInView ? styles.visible : ''}>
      {/* Content */}
    </section>
  );
}
```

### Step 3: Add CSS
**File**: `src/pages/index.module.css`

```css
.coreSystemsSection {
  opacity: 0;
  transition: opacity 0.6s ease-out;
}

.coreSystemsSection.visible {
  opacity: 1;
}
```

---

## Example 9: Change Sensor Network Visualization

### Modify SVG in SensorMapSection
**File**: `src/pages/index.tsx` (lines 242-320)

Add more nodes or change positions:

```typescript
<svg className={styles.sensorNetwork} viewBox="0 0 600 400">
  {/* Add more circles and lines */}
  <circle cx="200" cy="100" r="12" fill="rgba(0,255,255,0.6)"/>
  <text x="200" y="105" textAnchor="middle" fontSize="8" fill="#fff">NW</text>

  {/* Add more connection lines */}
  <line x1="300" y1="200" x2="200" y2="100" stroke="#00ffff" strokeWidth="2" opacity="0.5"/>
</svg>
```

---

## Example 10: Add FAQ Section

### Complete Section Example
**File**: `src/pages/index.tsx`

```typescript
function FAQSection() {
  const faqs = [
    {
      id: 1,
      question: 'Is this guide for beginners?',
      answer: 'Yes! We start with fundamentals and progress to advanced topics.',
    },
    {
      id: 2,
      question: 'Do I need prior robotics experience?',
      answer: 'No prior experience needed, just basic programming knowledge.',
    },
  ];

  const [expandedId, setExpandedId] = useState<number | null>(null);

  return (
    <section className={styles.faqSection}>
      <div className={styles.sectionLabel}>FAQ</div>
      <h2 className={styles.sectionTitle}>FREQUENTLY ASKED QUESTIONS</h2>
      <div className={styles.faqList}>
        {faqs.map((faq) => (
          <div key={faq.id} className={styles.faqItem}>
            <button
              className={styles.faqQuestion}
              onClick={() => setExpandedId(expandedId === faq.id ? null : faq.id)}
            >
              {faq.question}
              <span className={styles.faqToggle}>
                {expandedId === faq.id ? '−' : '+'}
              </span>
            </button>
            {expandedId === faq.id && (
              <div className={styles.faqAnswer}>{faq.answer}</div>
            )}
          </div>
        ))}
      </div>
    </section>
  );
}
```

---

## Summary of Common Customizations

| Customization | File | Lines | Complexity |
|--------------|------|-------|-----------|
| Change colors | index.module.css | Throughout | Low |
| Update text | index.tsx | Various | Low |
| Add card | index.tsx | 87-135 | Low |
| Add section | Both files | New | Medium |
| Change layout | index.module.css | Various | Medium |
| Add animation | index.module.css | Add keyframes | Medium |
| Modify styling | index.module.css | Target class | Low |
| Change links | index.tsx | Lines with `<Link>` | Low |

---

These examples should give you a solid starting point for customizing the landing page to fit your specific needs!
