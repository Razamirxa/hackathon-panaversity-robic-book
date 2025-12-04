import React, { useState } from 'react';
import './OnboardingModal.css';

interface HardwareProfile {
  hardware_type: string;
  hardware_specs: string;
  has_gpu: boolean;
  has_robot_kit: boolean;
  robot_kit_type: string;
}

interface ExperienceProfile {
  education_level: string;
  coding_experience: string;
  robotics_experience: string;
  ml_experience: string;
  ros_experience: string;
  programming_languages: string[];
}

interface GoalsProfile {
  learning_goals: string[];
  preferred_pace: string;
  time_commitment: string;
  areas_of_interest: string[];
}

interface OnboardingModalProps {
  isOpen: boolean;
  onComplete: () => void;
  userName?: string;
}

const PROGRAMMING_LANGUAGES = [
  { id: 'python', label: 'Python' },
  { id: 'cpp', label: 'C/C++' },
  { id: 'javascript', label: 'JavaScript/TypeScript' },
  { id: 'rust', label: 'Rust' },
  { id: 'java', label: 'Java' },
  { id: 'matlab', label: 'MATLAB' },
];

const LEARNING_GOALS = [
  { id: 'simulation', label: 'Build robot simulations' },
  { id: 'physical_robots', label: 'Work with physical robots' },
  { id: 'research', label: 'Academic research' },
  { id: 'career', label: 'Career advancement' },
  { id: 'hobby', label: 'Personal hobby/learning' },
  { id: 'startup', label: 'Start a robotics company' },
];

const AREAS_OF_INTEREST = [
  { id: 'humanoids', label: 'Humanoid Robots' },
  { id: 'manipulation', label: 'Robotic Manipulation' },
  { id: 'navigation', label: 'Autonomous Navigation' },
  { id: 'vision', label: 'Computer Vision' },
  { id: 'nlp', label: 'Natural Language Processing' },
  { id: 'reinforcement_learning', label: 'Reinforcement Learning' },
  { id: 'simulation', label: 'Simulation & Digital Twins' },
  { id: 'hardware', label: 'Robot Hardware Design' },
];

export default function OnboardingModal({ isOpen, onComplete, userName }: OnboardingModalProps) {
  const [step, setStep] = useState(1);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState('');

  // Hardware Profile
  const [hardware, setHardware] = useState<HardwareProfile>({
    hardware_type: '',
    hardware_specs: '',
    has_gpu: false,
    has_robot_kit: false,
    robot_kit_type: '',
  });

  // Experience Profile
  const [experience, setExperience] = useState<ExperienceProfile>({
    education_level: '',
    coding_experience: '',
    robotics_experience: 'none',
    ml_experience: 'none',
    ros_experience: 'none',
    programming_languages: [],
  });

  // Goals Profile
  const [goals, setGoals] = useState<GoalsProfile>({
    learning_goals: [],
    preferred_pace: 'self_paced',
    time_commitment: 'few_hours_week',
    areas_of_interest: [],
  });

  const toggleArrayItem = (
    array: string[],
    item: string,
    setFn: (arr: string[]) => void
  ) => {
    if (array.includes(item)) {
      setFn(array.filter((i) => i !== item));
    } else {
      setFn([...array, item]);
    }
  };

  const handleSubmit = async () => {
    setIsSubmitting(true);
    setError('');

    try {
      const response = await fetch('http://localhost:8000/api/auth/onboarding', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify({
          hardware,
          experience,
          goals,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Failed to save profile');
      }

      onComplete();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSubmitting(false);
    }
  };

  const canProceed = () => {
    switch (step) {
      case 1:
        return hardware.hardware_type !== '';
      case 2:
        return (
          experience.education_level !== '' &&
          experience.coding_experience !== ''
        );
      case 3:
        return goals.learning_goals.length > 0;
      default:
        return false;
    }
  };

  if (!isOpen) return null;

  return (
    <div className="onboarding-overlay">
      <div className="onboarding-modal">
        <div className="onboarding-header">
          <h2>👋 Welcome{userName ? `, ${userName}` : ''}!</h2>
          <p>Let's personalize your learning experience</p>
          <div className="step-indicator">
            <div className={`step ${step >= 1 ? 'active' : ''}`}>1. Hardware</div>
            <div className={`step ${step >= 2 ? 'active' : ''}`}>2. Experience</div>
            <div className={`step ${step >= 3 ? 'active' : ''}`}>3. Goals</div>
          </div>
        </div>

        <div className="onboarding-content">
          {/* Step 1: Hardware */}
          {step === 1 && (
            <div className="step-content">
              <h3>🖥️ What hardware do you have access to?</h3>
              <p className="step-description">
                This helps us tailor examples and exercises to your setup.
              </p>

              <div className="option-group">
                <label className="option-label">Primary Development Environment</label>
                <div className="option-cards">
                  {[
                    { id: 'laptop', icon: '💻', label: 'Laptop/Desktop' },
                    { id: 'raspberry_pi', icon: '🍓', label: 'Raspberry Pi' },
                    { id: 'jetson', icon: '🚀', label: 'NVIDIA Jetson' },
                    { id: 'cloud_only', icon: '☁️', label: 'Cloud Only' },
                    { id: 'other', icon: '🔧', label: 'Other' },
                  ].map((option) => (
                    <button
                      key={option.id}
                      className={`option-card ${hardware.hardware_type === option.id ? 'selected' : ''}`}
                      onClick={() =>
                        setHardware({ ...hardware, hardware_type: option.id })
                      }
                    >
                      <span className="option-icon">{option.icon}</span>
                      <span className="option-text">{option.label}</span>
                    </button>
                  ))}
                </div>
              </div>

              <div className="checkbox-group">
                <label className="checkbox-label">
                  <input
                    type="checkbox"
                    checked={hardware.has_gpu}
                    onChange={(e) =>
                      setHardware({ ...hardware, has_gpu: e.target.checked })
                    }
                  />
                  <span>I have GPU acceleration (NVIDIA CUDA)</span>
                </label>

                <label className="checkbox-label">
                  <input
                    type="checkbox"
                    checked={hardware.has_robot_kit}
                    onChange={(e) =>
                      setHardware({ ...hardware, has_robot_kit: e.target.checked })
                    }
                  />
                  <span>I have a robot kit or physical robot</span>
                </label>
              </div>

              {hardware.has_robot_kit && (
                <div className="input-group">
                  <label>What type of robot kit do you have?</label>
                  <input
                    type="text"
                    placeholder="e.g., TurtleBot 4, Boston Dynamics Spot, Custom Arduino robot"
                    value={hardware.robot_kit_type}
                    onChange={(e) =>
                      setHardware({ ...hardware, robot_kit_type: e.target.value })
                    }
                  />
                </div>
              )}

              <div className="input-group">
                <label>Additional hardware details (optional)</label>
                <textarea
                  placeholder="e.g., 32GB RAM, RTX 4080, specific sensors..."
                  value={hardware.hardware_specs}
                  onChange={(e) =>
                    setHardware({ ...hardware, hardware_specs: e.target.value })
                  }
                />
              </div>
            </div>
          )}

          {/* Step 2: Experience */}
          {step === 2 && (
            <div className="step-content">
              <h3>📚 Tell us about your background</h3>
              <p className="step-description">
                We'll adjust content complexity and explanations based on your experience.
              </p>

              <div className="option-group">
                <label className="option-label">Education Level</label>
                <select
                  value={experience.education_level}
                  onChange={(e) =>
                    setExperience({ ...experience, education_level: e.target.value })
                  }
                >
                  <option value="">Select your level...</option>
                  <option value="high_school">High School</option>
                  <option value="undergraduate">Undergraduate</option>
                  <option value="graduate">Graduate (Masters/PhD)</option>
                  <option value="professional">Working Professional</option>
                </select>
              </div>

              <div className="option-group">
                <label className="option-label">Coding Experience</label>
                <div className="option-cards horizontal">
                  {[
                    { id: 'beginner', label: 'Beginner', desc: 'Just starting out' },
                    { id: 'intermediate', label: 'Intermediate', desc: '1-3 years' },
                    { id: 'advanced', label: 'Advanced', desc: '3-5+ years' },
                    { id: 'expert', label: 'Expert', desc: 'Professional developer' },
                  ].map((option) => (
                    <button
                      key={option.id}
                      className={`option-card small ${experience.coding_experience === option.id ? 'selected' : ''}`}
                      onClick={() =>
                        setExperience({ ...experience, coding_experience: option.id })
                      }
                    >
                      <span className="option-text">{option.label}</span>
                      <span className="option-desc">{option.desc}</span>
                    </button>
                  ))}
                </div>
              </div>

              <div className="experience-grid">
                <div className="option-group">
                  <label className="option-label">Robotics Experience</label>
                  <select
                    value={experience.robotics_experience}
                    onChange={(e) =>
                      setExperience({ ...experience, robotics_experience: e.target.value })
                    }
                  >
                    <option value="none">None</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                </div>

                <div className="option-group">
                  <label className="option-label">Machine Learning Experience</label>
                  <select
                    value={experience.ml_experience}
                    onChange={(e) =>
                      setExperience({ ...experience, ml_experience: e.target.value })
                    }
                  >
                    <option value="none">None</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                </div>

                <div className="option-group">
                  <label className="option-label">ROS/ROS2 Experience</label>
                  <select
                    value={experience.ros_experience}
                    onChange={(e) =>
                      setExperience({ ...experience, ros_experience: e.target.value })
                    }
                  >
                    <option value="none">None</option>
                    <option value="beginner">Beginner</option>
                    <option value="intermediate">Intermediate</option>
                    <option value="advanced">Advanced</option>
                  </select>
                </div>
              </div>

              <div className="option-group">
                <label className="option-label">Programming Languages You Know</label>
                <div className="chip-group">
                  {PROGRAMMING_LANGUAGES.map((lang) => (
                    <button
                      key={lang.id}
                      className={`chip ${experience.programming_languages.includes(lang.id) ? 'selected' : ''}`}
                      onClick={() =>
                        toggleArrayItem(
                          experience.programming_languages,
                          lang.id,
                          (arr) =>
                            setExperience({ ...experience, programming_languages: arr })
                        )
                      }
                    >
                      {lang.label}
                    </button>
                  ))}
                </div>
              </div>
            </div>
          )}

          {/* Step 3: Goals */}
          {step === 3 && (
            <div className="step-content">
              <h3>🎯 What are your learning goals?</h3>
              <p className="step-description">
                Help us recommend the right content and learning path for you.
              </p>

              <div className="option-group">
                <label className="option-label">What do you want to achieve? (Select all that apply)</label>
                <div className="chip-group large">
                  {LEARNING_GOALS.map((goal) => (
                    <button
                      key={goal.id}
                      className={`chip ${goals.learning_goals.includes(goal.id) ? 'selected' : ''}`}
                      onClick={() =>
                        toggleArrayItem(goals.learning_goals, goal.id, (arr) =>
                          setGoals({ ...goals, learning_goals: arr })
                        )
                      }
                    >
                      {goal.label}
                    </button>
                  ))}
                </div>
              </div>

              <div className="option-group">
                <label className="option-label">Areas of Interest (Select all that apply)</label>
                <div className="chip-group large">
                  {AREAS_OF_INTEREST.map((area) => (
                    <button
                      key={area.id}
                      className={`chip ${goals.areas_of_interest.includes(area.id) ? 'selected' : ''}`}
                      onClick={() =>
                        toggleArrayItem(goals.areas_of_interest, area.id, (arr) =>
                          setGoals({ ...goals, areas_of_interest: arr })
                        )
                      }
                    >
                      {area.label}
                    </button>
                  ))}
                </div>
              </div>

              <div className="option-group">
                <label className="option-label">Preferred Learning Pace</label>
                <div className="option-cards horizontal">
                  {[
                    { id: 'self_paced', label: 'Self-Paced', desc: 'Learn at my own speed' },
                    { id: 'structured', label: 'Structured', desc: 'Weekly milestones' },
                    { id: 'intensive', label: 'Intensive', desc: 'Accelerated learning' },
                  ].map((option) => (
                    <button
                      key={option.id}
                      className={`option-card small ${goals.preferred_pace === option.id ? 'selected' : ''}`}
                      onClick={() => setGoals({ ...goals, preferred_pace: option.id })}
                    >
                      <span className="option-text">{option.label}</span>
                      <span className="option-desc">{option.desc}</span>
                    </button>
                  ))}
                </div>
              </div>

              <div className="option-group">
                <label className="option-label">Time Commitment</label>
                <select
                  value={goals.time_commitment}
                  onChange={(e) => setGoals({ ...goals, time_commitment: e.target.value })}
                >
                  <option value="few_hours_week">A few hours per week</option>
                  <option value="part_time">Part-time (10-20 hrs/week)</option>
                  <option value="full_time">Full-time (30+ hrs/week)</option>
                </select>
              </div>
            </div>
          )}
        </div>

        {error && <div className="onboarding-error">{error}</div>}

        <div className="onboarding-footer">
          {step > 1 && (
            <button
              className="btn-secondary"
              onClick={() => setStep(step - 1)}
              disabled={isSubmitting}
            >
              ← Back
            </button>
          )}
          <div className="spacer" />
          {step < 3 ? (
            <button
              className="btn-primary"
              onClick={() => setStep(step + 1)}
              disabled={!canProceed()}
            >
              Continue →
            </button>
          ) : (
            <button
              className="btn-primary"
              onClick={handleSubmit}
              disabled={!canProceed() || isSubmitting}
            >
              {isSubmitting ? 'Saving...' : '🚀 Start Learning'}
            </button>
          )}
        </div>
      </div>
    </div>
  );
}
