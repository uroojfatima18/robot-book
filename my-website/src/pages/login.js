import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../components/AuthContext';
import styles from './auth.module.css';

export default function Login() {
  const [formData, setFormData] = useState({ email: '', password: '' });
  const { login } = useAuth();
  const history = useHistory();

  const handleSubmit = (e) => {
    e.preventDefault();
    // Simulate API delay
    const success = login(formData.email, formData.password);
    if (success) {
      alert('IDENTITY VERIFIED. WELCOME BACK, OPERATOR.');
      history.push('/');
    }
  };

  return (
    <Layout title="Login" description="Access Neural Link">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.authHeader}>
            <h1 className={styles.glitchTitle} data-text="ACCESS_NODE">ACCESS_NODE</h1>
            <p>Identify yourself to proceed.</p>
          </div>
          
          <form onSubmit={handleSubmit} className={styles.authForm}>
            <div className={styles.inputGroup}>
              <label>NEURAL_ID (EMAIL)</label>
              <input 
                type="email" 
                required 
                placeholder="user@net.protocol"
                value={formData.email}
                onChange={(e) => setFormData({...formData, email: e.target.value})}
              />
              <div className={styles.inputBorder}></div>
            </div>
            
            <div className={styles.inputGroup}>
              <label>Passkey</label>
              <input 
                type="password" 
                required 
                placeholder="••••••••"
                value={formData.password}
                onChange={(e) => setFormData({...formData, password: e.target.value})}
              />
              <div className={styles.inputBorder}></div>
            </div>

            <button type="submit" className={styles.authButton}>
              <span className={styles.btnText}>INITIATE_LINK</span>
              <span className={styles.btnIcon}>⚡</span>
            </button>
          </form>

          <div className={styles.authFooter}>
            <p>New node? <Link to="/signup">Initialize Protocol</Link></p>
          </div>
        </div>
      </div>
    </Layout>
  );
}
