"""add auth fields to users

Revision ID: add_auth_fields
Revises: 4607d1cf9799
Create Date: 2025-12-04

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = 'add_auth_fields'
down_revision: Union[str, None] = '4607d1cf9799'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # Add new columns to users table
    op.add_column('users', sa.Column('name', sa.String(255), nullable=True))
    op.add_column('users', sa.Column('password_hash', sa.String(255), nullable=True))
    op.add_column('users', sa.Column('email_verified', sa.Boolean(), server_default='false', nullable=True))
    op.add_column('users', sa.Column('updated_at', sa.DateTime(timezone=True), server_default=sa.func.now(), nullable=True))
    
    # Update existing users with null email to have email based on username
    op.execute("UPDATE users SET email = username || '@example.com' WHERE email IS NULL")
    
    # Make username optional (allow null)
    op.alter_column('users', 'username', nullable=True)
    
    # Make email required (not null) - now safe since we filled nulls
    op.alter_column('users', 'email', nullable=False)
    
    # Create index on email if it doesn't exist
    try:
        op.create_index('ix_users_email', 'users', ['email'], unique=True)
    except:
        pass  # Index might already exist


def downgrade() -> None:
    # Remove index
    op.drop_index('ix_users_email', table_name='users')
    
    # Revert column changes
    op.alter_column('users', 'username', nullable=False)
    op.alter_column('users', 'email', nullable=True)
    
    # Remove columns
    op.drop_column('users', 'updated_at')
    op.drop_column('users', 'email_verified')
    op.drop_column('users', 'password_hash')
    op.drop_column('users', 'name')
